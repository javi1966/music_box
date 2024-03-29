import math, struct, winsound

sample_rate = 32000
##wavetable_fundamental_frequency = sample_rate / 1000
wavetable_fundamental_frequency = 262 # tbd: should lead to single amplitude steps
print wavetable_fundamental_frequency

wavetable = [ 0 ] * ( sample_rate / wavetable_fundamental_frequency )
print len( wavetable ), 'Bytes'
for i in range( len( wavetable ) ):
    wavetable[ i ] = int( round( 127 * math.sin( 2 * math.pi * i / len( wavetable ) ) ) )
##print wavetable

wt_attack = [ 0, 0, 0, 0, 0, 0, -1, -2, -2, -3, -2, -2, -1, 0, 0, 0,
              0, 0, -1, -2, -3, -3, -4, -4, -3, -2, -1, 0, 0, 1, 0, 0,
              0, -1, -2, -3, -3, -2, 0, 0, 2, 4, 5, 5, 5, 3, 1, 0,
              -2, -3, -4, -3, -2, 0, 1, 2, 3, 2, 0, -2, -6, -11, -15, -18,
              -19, -19, -16, -11, -5, 1, 8, 15, 20, 23, 24, 23, 20, 17, 13, 10,
              7, 6, 6, 8, 10, 13, 16, 18, 20, 20, 20, 19, 18, 18, 18, 18,
              19, 21, 24, 26, 27, 28, 27, 25, 22, 17, 11, 5, 0, -5, -9, -12,
              -13, -13, -11, -9, -5, -1, 1, 4, 6, 6, 4, 0, -5, -12, -21, -30,
              -39, -48, -56, -62, -66, -69, -70, -71, -70, -70, -70, -71, -72, -75, -77, -79,
              -80, -78, -75, -69, -61, -50, -38, -26, -13, -2, 7, 15, 21, 25, 28, 30,
              33, 36, 40, 44, 49, 55, 59, 61, 62, 60, 56, 49, 42, 34, 27, 22,
              20, 21, 25, 33, 42, 52, 63, 72, 80, 86, 89, 90, 89, 87, 84, 81,
              79, 77, 75, 73, 72, 69, 66, 61, 55, 48, 39, 31, 22, 14, 6, 0,
              -5, -10, -14, -17, -20, -22, -24, -26, -28, -29, -30, -31, -32, -33, -34, -35,
              -36, -37, -39, -41, -44, -48, -52, -57, -63, -70, -77, -85, -94, -102, -110, -116,
              -121, -124, -124, -121, -116, -109, -99, -89, -78, -68, -60, -53, -48, -45, -43, -43,
              -43, -42, -41, -40, -37, -33, -29, -25, -22, -19, -17, -16, -15, -14, -12, -9,
              -4, 2, 11, 22, 34, 46, 57, 67, 75, 81, 84, 86, 87, 86, 86, 85,
              85, 85, 86, 86, 87, 86, 84, 82, 78, 73, 69, 64, 60, 58, 56, 57,
              59, 62, 66, 70, 75, 79, 83, 86, 89, 91, 92, 92, 92, 90, 86, 81,
              73, 63, 52, 38, 24, 9, -5, -19, -31, -42, -50, -57, -61, -64, -66, -67,
              -68, -68, -68, -67, -66, -64, -61, -56, -52, -46, -41, -36, -32, -29, -28, -28,
              -30, -34, -39, -44, -50, -56, -62, -68, -74, -79, -84, -90, -95, -100, -104, -107,
              -108, -108, -107, -103, -98, -92, -84, -76, -67, -58, -49, -40, -32, -24, -16, -8,
              -1, 5, 11, 16, 20, 22, 22, 21, 18, 14, 10, 5, 1, -1, -3, -3,
              -2, 0, 1, 5, 9, 12, 16, 20, 24, 28, 34, 40, 47, 55, 64, 72,
              81, 89, 96, 101, 105, 108, 108, 108, 107, 105, 103, 101, 100, 98, 96, 93,
              90, 87, 83, 79, 75, 70, 66, 62, 59, 56, 55, 53, 52, 51, 50, 48,
              46, 44, 42, 39, 37, 34, 31, 28, 24, 20, 15, 10, 3, -3, -12, -21,
              -30, -39, -48, -57, -65, -72, -79, -85, -90, -95, -99, -103, -106, -109, -110, -112,
              -112, -112, -111, -110, -109, -107, -106, -105, -104, -104, -103, -102, -101, -99, -97, -94,
              -90, -86, -82, -78, -75, -72, -69, -67, -65, -63, -61, -58, -54, -50, -44, -38,
              -31, -23, -16, -10, -4, 1, 6, 10, 14, 18, 23, 27, 32, 38, 43, 49,
              54, 59, 63, 67, 70, 73, 75, 77, 79, 81, 83, 85, 88, 91, 93, 96,
              99, 101, 103, 105, 107, 109, 110, 111, 112, 112, 112, 111, 109, 107, 104, 100,
              96, 92, 88, 83, 79, 75, 70, 65, 60, 55, 49, 43, 36, 30, 24, 18,
              12, 7, 3, 0, -3, -6, -8, -11, -14, -17, -20, -24, -28, -32, -37, -42,
              -46, -51, -55, -59, -63, -67, -71, -75, -79, -82, -85, -88, -91, -93, -95, -97,
              -98, -100, -101, -102, -103, -104, -104, -104, -103, -101, -99, -96, -93, -89, -86, -83,
              -80, -78, -76, -75, -74, -73, -72, -71, -70, -68, -65, -63, -59, -56, -52, -48,
              -44, -40, -35, -30, -24, -18, -11, -4, 3, 11, 19, 27, 34, 41, 48, 53,
              58, 63, 67, 71, 75, 79, 82, 86, 89, 92, 94, 95, 96, 97, 98, 98,
              99, 100, 101, 103, 105, 108, 110, 112, 114, 115, 115, 114, 113, 110, 107, 103,
              99, 95, 91, 87, 83, 79, 74, 70, 65, 59, 53, 47, 41, 35, 29, 24,
              19, 14, 11, 7, 4, 1, -1, -5, -9, -14, -19, -25, -31, -37, -43, -50,
              -56, -62, -68, -73, -78, -83, -88, -92, -97, -101, -105, -109, -112, -115, -117, -119,
              -120, -121, -120, -119, -118, -116, -114, -112, -109, -106, -104, -101, -99, -96, -94, -91,
              -88, -85, -81, -78, -74, -70, -66, -62, -57, -53, -49, -45, -41, -37, -32, -27,
              -22, -16, -10, -4, 1, 8, 14, 20, 25, 31, 36, 41, 46, 50, 55, 60,
              64, 69, 73, 77, 81, 85, 88, 90, 93, 95, 97, 99, 101, 103, 106, 109,
              112, 115, 118, 121, 123, 125, 126, 126, 125, 125, 123, 121, 119, 117, 115, 112,
              110, 107, 103, 99, 94, 89, 83, 76, 69, 62, 54, 47, 39, 31, 24, 16,
              9, 2, -4, -11, -18, -25, -31, -38, -44, -51, -57, -62, -68, -73, -79, -84,
              -88, -93, -97, -102, -106, -109, -113, -116, -119, -121, -123, -124, -125, -126, -127, -127,
              -127, -127, -126, -125, -125, -123, -122, -120, -118, -116, -114, -111, -107, -104, -100, -95,
              -91, -86, -80, -75, -69, -63, -58, -52, -46, -40, -34, -28, -22, -16, -10, -4 ]

wt = wt_attack + wavetable

envelope_byte_count = 1024 # tbd: should be long enough for small decay steps
envelope_table = [ 0 ] * envelope_byte_count
for i in range( envelope_byte_count ):
    envelope_table[ i ] = int( round( 255. / math.exp( i / 163. ) ) ) # tbd: should match output bit-depth for single amplitude steps at the end
##print envelope_table

data = ''
phase_accu_int = 0

pot = 10 # power of two
increment = ( 1 << pot ) * 1940 / wavetable_fundamental_frequency
print increment

envelope_position = 0
for i in range( sample_rate * 1 ): # seconds
    phase_accu_int += increment
    if phase_accu_int >= len( wt ) << pot:
        phase_accu_int -= len( wavetable ) << pot
    phase_accu = phase_accu_int >> pot
    value = ( envelope_table[ envelope_position ] * wt[ phase_accu ] ) >> 0
    data += struct.pack( '<h', value )
    if ( phase_accu_int >= len( wt_attack ) << pot and
         envelope_position < len( envelope_table ) - 1 and
         i % int( 2 * sample_rate / envelope_byte_count ) == 0 ):
        envelope_position += 1

def __header( samplesPerSec, dataSize ):
    headerSize = 44
    RIFFChunkID = 'RIFF'
    RIFFChunkSize = dataSize + headerSize - 8
    RIFFType = 'WAVE'
    fmtChunkID = 'fmt '
    fmtChunkSize = 16 # 16 Byte
    wFormatTag = 0x0001 # WAVE_FORMAT_PCM
    wChannels = 1
    dwSamplesPerSec = samplesPerSec
    nBitsPerSample = 16
    wBlockAlign = wChannels * ( ( nBitsPerSample + 7 ) // 8 )
    dwAvgBytesPerSec = samplesPerSec * wBlockAlign
    dataChunkID = 'data'
    dataChunkSize = dataSize
    header = struct.pack( '<4sL4s4s', RIFFChunkID, RIFFChunkSize, RIFFType, fmtChunkID )
    header += struct.pack( '<LHHLL', fmtChunkSize, wFormatTag, wChannels, dwSamplesPerSec, dwAvgBytesPerSec )
    header += struct.pack( '<HH4sL', wBlockAlign, nBitsPerSample, dataChunkID, dataChunkSize )
    assert len( header ) == headerSize, 'Error: Actual headerSize (%s) differs from assumed one (%s).' % ( len( header ), headerSize )
    return header

header = __header( sample_rate, len( data ) )
winsound.PlaySound( header + data, winsound.SND_MEMORY | winsound.SND_NOWAIT )
f = open( 'dds.wav', 'wb' ); f.write( header + data ); f.close()
