def _rotation_order(_saved_channels):
    saved_rotations = []

    for chan in _saved_channels:
        if chan.lower().find('rotation') != -1:
            saved_rotations.append(chan)
    
    rotation_string = ' '.join(saved_rotations)
    return (ord(rotation_string[0]) - ord('X') + 1) * 1 + (ord(rotation_string[10]) - ord('X') + 1) * 2 + (ord(rotation_string[20]) - ord('X') + 1) * 4 - 10
