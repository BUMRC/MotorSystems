def lower(s):
    newS = ""
    for c in s:
        if ord(c) >= ord('A') and ord(c) <= ord('Z'):
            newS += chr(ord(c) + ord('a') - ord('A'))
        else:
            newS += c
    return newS

print(lower("Hello"))
