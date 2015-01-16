import sys

def ascii_to_int(char):
    if char >= '0' and char <= '9':
        return ord(char) - ord('0')
    elif char >= 'a' and char <= 'f':
        return 10 + (ord(char) - ord('a'))
    elif char >= 'A' and char <= 'F':
        return 10 + (ord(char) - ord('A'))
    else:
        raise AssertionError()

def get_byte_from_ascii_str(msg, idx):
    msg_list = list(msg)
    if idx < (len(msg_list) / 2):
        return ascii_to_int(msg_list[idx * 2]) * 16 + ascii_to_int(msg_list[idx * 2 + 1])
    else:
        raise AssertionError()

def get_short_from_ascii_str(msg, idx):
    return get_byte_from_ascii_str(msg, idx) + get_byte_from_ascii_str(msg, idx + 1) * 256

def get_int_from_ascii_str(msg, idx):
    return get_byte_from_ascii_str(msg, idx) + get_byte_from_ascii_str(msg, idx + 1) * 256 + get_byte_from_ascii_str(msg, idx + 2) * 256 * 256 + get_byte_from_ascii_str(msg, idx + 3) * 256 * 256 * 256

def quad_to_ascii(u4):
    if u4 < 10:
        return chr(ord('0') + u4)
    else:
        return chr(ord('a') + u4 - 10)

def uint8_to_ascii(u8):
    return quad_to_ascii(u8 / 16) + quad_to_ascii(u8 % 16)

def int_to_ascii(integer, length):
    ret = ""
    for i in range(0, length):
        ret = ret + uint8_to_ascii(integer % 256)
        integer = integer / 256
    ret_reverse = ""
    n = 2
    for i in range(0, len(ret), n):
        ret_reverse = ret_reverse + ret[(len(ret) - i - n) : (len(ret) - i)]
    return ret_reverse
