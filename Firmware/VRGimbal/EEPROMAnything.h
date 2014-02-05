template <class T> int EEPROM_writeAnything(int ee, const T& value, byte * crc)
{
    const byte* p = (const byte*)(const void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
          EEPROM.write(ee++, *p++);

    if (crc)
    	EEPROM.write(ee++, (*crc));
    return i;
}

template <class T> int EEPROM_readAnything(int ee, T& value, byte * crc)
{
    byte* p = (byte*)(void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
          *p++ = EEPROM.read(ee++);

    if (crc)
        *crc = EEPROM.read(ee++);
    return i;
}


