import os
import struct
import binascii
#import StringIO
from collections import namedtuple
import io
import jinja2
from elftools.elf.elffile import ELFFile


class PackFlashAlgo(object):
    REQUIRED_SYMBOLS = (
        "Init",
        "UnInit",
        "EraseSector",
        "ProgramPage",
    )

    EXTRA_SYMBOLS = (
        "BlankCheck",
        "EraseChip",
        "Verify",
        "Read",
    )

    def __init__(self, data):
        
        self.elf = ElfFileSimple(data)

        self.flash_info = PackFlashInfo(self.elf)
        self.flash_start     = self.flash_info.start
        self.flash_size      = self.flash_info.size
        self.flash_page_size = self.flash_info.page_size


        self.symbols = {}
        for symbol in self.REQUIRED_SYMBOLS:
            if symbol not in self.elf.symbols: raise Exception("Missing symbol %s" % symbol)
            self.symbols[symbol] = self.elf.symbols[symbol].value
        for symbol in self.EXTRA_SYMBOLS:
            if symbol not in self.elf.symbols: self.symbols[symbol] = 0xFFFFFFFF
            else:                              self.symbols[symbol] = self.elf.symbols[symbol].value


        
        ro_rw_zi = [None, None, None]
        for section in self.elf.iter_sections():
            for i, name_and_type in enumerate((("PrgCode", "SHT_PROGBITS"),
                                               ("PrgData", "SHT_PROGBITS"),
                                               ("PrgData", "SHT_NOBITS"),)):
                if name_and_type != (section.name, section["sh_type"]): continue
                if ro_rw_zi[i] is not None: raise Exception("Duplicated section")

                ro_rw_zi[i] = section


        s_ro, s_rw, s_zi = ro_rw_zi
        if s_rw is not None and s_zi is None:
            s_zi = {
                "sh_addr": s_rw["sh_addr"] + s_rw["sh_size"],
                "sh_size": 0
            }
        
        if s_ro is None: raise Exception("RO section is missing")
        if s_rw is None: raise Exception("RW section is missing")
        if s_zi is None: raise Exception("ZI section is missing")
        if s_ro["sh_addr"] != 0:
            raise Exception("RO section does not start at address 0")
        if s_ro["sh_addr"] + s_ro["sh_size"] != s_rw["sh_addr"]:
            raise Exception("RW section does not follow RO section")
        if s_rw["sh_addr"] + s_rw["sh_size"] != s_zi["sh_addr"]:
            raise Exception("ZI section does not follow RW section")

        self.ro_start = s_ro["sh_addr"]
        self.ro_size  = s_ro["sh_size"]
        self.rw_start = s_rw["sh_addr"]
        self.rw_size  = s_rw["sh_size"]
        self.zi_start = s_zi["sh_addr"]
        self.zi_size  = s_zi["sh_size"]

        
        algo_size = s_ro["sh_size"] + s_rw["sh_size"] + s_zi["sh_size"]
        self.algo_data = bytearray(algo_size)
        for section in (s_ro, s_rw):
            start = section["sh_addr"]
            size  = section["sh_size"]
            assert len(section.data()) == size
            self.algo_data[start : start+size] = section.data()

    def format_algo_data(self, spaces, group_size, fmt):
        
        padding = " " * spaces
        if fmt == "hex":
            blob = binascii.b2a_hex(self.algo_data)
            line_list = []
            for i in xrange(0, len(blob), group_size):
                line_list.append('"' + blob[i:i + group_size] + '"')
            return ("\n" + padding).join(line_list)
        elif fmt == "c":
            blob = self.algo_data[:]
            pad_size = 0 if len(blob) % 4 == 0 else 4 - len(blob) % 4
            blob = blob + b"\x00" * pad_size
            integer_list = struct.unpack("<" + "L" * int(len(blob) / 4), blob)
            line_list = []
            for pos in range(0, len(integer_list), group_size):
                group = ["0x%08X" % value for value in
                         integer_list[pos:pos + group_size]]
                line_list.append(", ".join(group))
            return (",\n" + padding).join(line_list)
        else:
            raise Exception("Unsupported format %s" % fmt)

    def process_template(self, template_path, output_path, data_dict=None):
        
        if data_dict is None: data_dict = {}
        
        assert "algo" not in data_dict, "algo already set by user data"
        data_dict["algo"] = self

        with open(template_path) as f:
            test1=f.read()
            template = jinja2.Template(test1)            
            with open(output_path, "w") as f:
                f.write(template.render(data_dict))


class PackFlashInfo(object):
    

    FLASH_DEVICE_STRUCT = "<H128sHLLLLBxxxLL"   
    FLASH_SECTOR_STRUCT = "<LL"                
    FLASH_DEVICE_STRUCT_SIZE = struct.calcsize(FLASH_DEVICE_STRUCT)
    FLASH_SECTOR_STRUCT_SIZE = struct.calcsize(FLASH_SECTOR_STRUCT)
    SECTOR_END = (0xFFFFFFFF, 0xFFFFFFFF)

    def __init__(self, elf):
        info = elf.symbols["FlashDevice"]
        info_start = info.value
        info_size  = self.FLASH_DEVICE_STRUCT_SIZE
        info_data  = elf.read(info_start, info_size)

        values = struct.unpack(self.FLASH_DEVICE_STRUCT, info_data)

        self.version          = values[0]
        self.name             = values[1].strip(b"\x00")
        self.type             = values[2]
        self.start            = values[3]
        self.size             = values[4]
        self.page_size        = values[5]
        self.value_empty      = values[7]
        self.prog_timeout_ms  = values[8]
        self.erase_timeout_ms = values[9]

        self.sector_info_list = []
        for i in range(512):   
            data = elf.read(info_start + info_size + self.FLASH_SECTOR_STRUCT_SIZE * i, self.FLASH_SECTOR_STRUCT_SIZE)
            size, addr = struct.unpack(self.FLASH_SECTOR_STRUCT, data)

            if (size, addr) == self.SECTOR_END: break

            self.sector_info_list.append((addr, size))

    def __str__(self):
        desc = ""
        desc += "Flash Device:" + os.linesep
        desc += "  name=%s"             % self.name             + os.linesep
        desc += "  version=0x%x"        % self.version          + os.linesep
        desc += "  type=%i"             % self.type             + os.linesep
        desc += "  start=0x%x"          % self.start            + os.linesep
        desc += "  size=0x%x"           % self.size             + os.linesep
        desc += "  page_size=0x%x"      % self.page_size        + os.linesep
        desc += "  value_empty=0x%x"    % self.value_empty      + os.linesep
        desc += "  prog_timeout_ms=%i"  % self.prog_timeout_ms  + os.linesep
        desc += "  erase_timeout_ms=%i" % self.erase_timeout_ms + os.linesep
        desc += "  sectors:" + os.linesep
        for sector_addr, sector_size in self.sector_info_list:
            desc += ("    addr=0x%x, size=0x%x" %(sector_addr, sector_size) + os.linesep)

        return desc


SymbolSimple = namedtuple("SymbolSimple", "name, value, size")

class ElfFileSimple(ELFFile):


    def __init__(self, data):
        super(ElfFileSimple, self).__init__(io.BytesIO(data))

        self.symbols = {}   
        for symbol in self.get_section_by_name(".symtab").iter_symbols():
            self.symbols[symbol.name] = SymbolSimple(symbol.name, symbol["st_value"], symbol["st_size"])

    def read(self, addr, size):    
        for segment in self.iter_segments():
            seg_addr = segment["p_paddr"]
            seg_size = min(segment["p_memsz"], segment["p_filesz"])
            
            if addr >= seg_addr and addr + size <= seg_addr + seg_size:
                start = addr - seg_addr
                return segment.data()[start : start+size]
            else:
                continue


if __name__ == '__main__':

    BLOB_HEADER = '0xE00ABE00, 0x062D780D, 0x24084068, 0xD3000040, 0x1E644058, 0x1C49D1FA, 0x2A001E52, 0x4770D1F2,'
    HEADER_SIZE = 0x20

    data_dict = {
        'prog_header': BLOB_HEADER,
        'header_size': HEADER_SIZE,
        'entry': 0x20000000,
        'stack_pointer': 0x20000000 + 4096,
    }

    for name in os.listdir(os.getcwd()):
        if os.path.isfile(name) and name.endswith('.FLM'):
            with open(name, 'rb') as f:
                algo = PackFlashAlgo(f.read())
                print(algo.flash_info)
                if 'algo' in data_dict: 
                    del data_dict['algo']
                algo.process_template('c_blob.tmpl', name.replace('.FLM', '.c'), data_dict)
