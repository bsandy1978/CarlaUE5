def hex_to_bgr(hex_code):
    hex_code = hex_code.lstrip('#')
    return tuple(int(hex_code[i:i+2], 16) for i in (4, 2, 0))
target_hex_code = "#01018e" 
target_bgr = hex_to_bgr(target_hex_code)
print(target_bgr)