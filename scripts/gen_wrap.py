import re
from pathlib import Path

from cv2 import split

with open(Path('include/LTSMC.h'), 'r') as f:
    codes = f.read()

all_functions = re.findall('.+ .+\(.*\);', str(codes))

py_wrap_functions = ''
module_added_functions = ''

for function in all_functions:
    if function.startswith('//'):
        continue
    return_type = function.split(' ')[0]
    function_name = re.findall('(?<=\ ).*(?=\()', str(function))[0]
    args_with_type = re.findall('(?<=\().*(?=\))', str(function))[0]
    args_without_type = ''
    arg_names = []
    for split_ret in args_with_type.split(','):
        arg_names.append(split_ret.split(' ')[-1])

    py_wrap_functions += f'''py::list py_{function_name}({args_with_type}) {{\n    py::list ret;\n    ret.append({function_name}({', '.join(arg_names)}));\n    return ret;\n}}\n\n'''

    module_added_functions += f"m.def(\"{function_name}\", &py_{function_name}, "
    for arg_name in arg_names:
        module_added_functions += f'py::arg("{arg_name}"), '
    module_added_functions = module_added_functions[:-2]
    module_added_functions += ');\n'

with open(Path('TMP.cpp'), 'w+') as f:
    f.write(py_wrap_functions)
    f.write(module_added_functions)
