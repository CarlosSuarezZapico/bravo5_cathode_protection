/**
 *    @file  bplprotocol_unite.cpp
 *    @brief bplprotocol library for bravo7 arm 
 *
 *    @author  Carlos Suarez Zapico, carlossuarezzapico@gmail.com
 *
 *    @internal
 *    Project  UNITE
 *    Created  16-May-2024
 *    Modification 16-May-2024
 *    Revision  ---
 *    Company  Heriot-Watt University / National Robotarium
 * ===============================================================
 */

#include "bravo_compliance/bravo_cpp/bravo_io_v2/bplprotocol_unite.h"


uint8_t cobs_encode_unite( uint8_t* input, uint8_t length, uint8_t * output)
{
	uint8_t read_index = 0;
	uint8_t write_index = 1;
	uint8_t code_index = 0;
    uint8_t code = 1;

    while(read_index < length) {
        if(input[read_index] == 0) {
            output[code_index] = code;
            code = 1;
            code_index = write_index++;
            read_index++;
        } else {
            output[write_index++] = input[read_index++];
            code++;
            if(code == 0xFF) {
                output[code_index] = code;
                code = 1;
                code_index = write_index++;
            }
        }
    }
    output[code_index] = code;
    return write_index;
}

uint8_t cobs_encode_unite2(const std::vector<uint8_t>& input, size_t length, std::vector<uint8_t>& output)
{
    uint8_t read_index = 0;
    uint8_t write_index = 1; // Start from 1 to leave space for the first code
    uint8_t code_index = 0; // Index where the next code will be inserted
    uint8_t code = 1; // Initialize code value

    // Ensure the output vector is clear and has an initial size set to at least input size + 1 for safety
    output.clear();
    output.resize(input.size() + 1);

    while(read_index < input.size()) {
        if(input[read_index] == 0) {
            output[code_index] = code;
            code = 1;
            code_index = write_index++;
            read_index++;
        } else {
            output[write_index++] = input[read_index++];
            code++;
            if(code == 0xFF) {
                output[code_index] = code;
                code = 1;
                code_index = write_index++;
            }
        }
    }
    output[code_index] = code;

    // Resize the output to the actual size used
    output.resize(write_index);

    return write_index; // Returns the size of the output
}

uint8_t cobs_decode_unite(const uint8_t* input, uint8_t length, uint8_t* output) {
	uint8_t read_index = 0;
	uint8_t write_index = 0;
    uint8_t code;
    uint8_t i;

    while(read_index < length) {
        code = input[read_index];
        if(read_index + code > length && code != 1) return 0;
        read_index++;
        for(i = 1; i < code; i++) output[write_index++] = input[read_index++];
        if(code != 0xFF && read_index != length)
        	output[write_index++] = '\0';
    }
    return write_index;
}

uint8_t cobs_decode_unite(const std::vector<uint8_t>& input, std::vector<uint8_t>& output) {
    uint8_t read_index = 0;
    uint8_t write_index = 0;
    uint8_t code;
    uint8_t i;
    uint8_t length = input.size();
    output.clear(); // Ensure the output vector is empty before starting decoding

    while(read_index < length) {
        code = input[read_index];
        // Check if the next code exceeds the length of the input
        if((read_index + code > length) && (code != 1)) return 0;

        read_index++;
        for(i = 1; i < code; i++) {
            if (write_index >= output.size()) {
                output.resize(write_index + 1); // Ensure there is space in the output vector
            }
            output[write_index++] = input[read_index++];
        }
        if(code != 0xFF && read_index != length) {
            if (write_index >= output.size()) {
                output.resize(write_index + 1); // Ensure there is space in the output vector
            }
            output[write_index++] = '\0';
        }
    }
    return write_index; // Return the size of the decoded data
}

// Return the CRC-8 of data[0..len-1] applied to the seed crc. This permits the
// calculation of a CRC a chunk at a time, using the previously returned value
// for the next seed. If data is NULL, then return the initial seed. See the
// test code for an example of the proper usage.
unsigned crc8(unsigned crc, unsigned char const *data, size_t len)
{
    if (data == NULL)
        return 0;
    crc &= 0xff;
    unsigned char const *end = data + len;
    while (data < end)
        crc = crc8_table[crc ^ *data++];
    return crc;
}

size_t encodePacketBare_unite(uint8_t* packet_buffer, uint8_t deviceID, uint8_t packetID, uint8_t* data, size_t length)
{
    uint8_t tempBuffer[MAX_PACKET_LENGTH];
    memset(tempBuffer, 0, MAX_PACKET_LENGTH);
    size_t totalLength = length + HEADER_SIZE;
    memcpy(tempBuffer, data, length);
    tempBuffer[totalLength-4] = packetID;
    tempBuffer[totalLength-3] = deviceID;
    tempBuffer[totalLength-2] = totalLength;
    tempBuffer[totalLength-1] = crc8(0xff, tempBuffer, totalLength-1);
    size_t packetLength = cobs_encode_unite(tempBuffer, totalLength, packet_buffer);

    return (size_t) packetLength;

}

size_t encodePacket_unite(std::vector<uint8_t>& packet_buffer, struct Packet* packet)
{
    std::vector<uint8_t> tempBuffer;
    tempBuffer.resize(MAX_PACKET_LENGTH);
    size_t length = packet->dataLength;
    size_t totalLength = length + HEADER_SIZE;
    memcpy(tempBuffer.data(), packet->data, length);
    tempBuffer[totalLength-4] = packet->packetID;
    tempBuffer[totalLength-3] = packet->deviceID;
    tempBuffer[totalLength-2] = totalLength;
    tempBuffer[totalLength-1] = crc8(0xff, tempBuffer.data(), totalLength-1);
    packet_buffer.clear(); // Clear existing contents
    packet_buffer.resize(totalLength);
    size_t packetLength = cobs_encode_unite2(tempBuffer, totalLength, packet_buffer);
    return packetLength;
}

size_t encodePacket_unite(uint8_t* packet_buffer, struct Packet* packet)
{
    uint8_t tempBuffer[MAX_PACKET_LENGTH];
    memset(tempBuffer, 0, MAX_PACKET_LENGTH);
    size_t length = packet->dataLength;
    size_t totalLength = length + HEADER_SIZE;
    memcpy(tempBuffer, packet->data, length);
    tempBuffer[totalLength-4] = packet->packetID;
    tempBuffer[totalLength-3] = packet->deviceID;
    tempBuffer[totalLength-2] = totalLength;
    tempBuffer[totalLength-1] = crc8(0xff, tempBuffer, totalLength-1);
    size_t packetLength = cobs_encode_unite(tempBuffer, totalLength, packet_buffer);
    return packetLength;
}

size_t encodeFloat(uint8_t* outputBuffer, float f)
{
    uint32_t asInt = *((uint32_t*)&f);
    int i;
	for (i = 0; i < 4; i++) {
		outputBuffer[i] = (asInt >> 8 * i) & 0xFF;
	}
	return 4;
}

uint8_t* encodeFloat(float f){
    uint8_t outputBuffer[4];
    uint32_t asInt = *((uint32_t*)&f);
    int i;
    for (i = 0; i < 4; i++) {
        outputBuffer[i] = (asInt >> 8 * i) & 0xFF;
    }
    return outputBuffer;
}

size_t encode_packet_float_list(uint8_t* packet_buffer, uint8_t packetID, uint8_t deviceID, float *floatList, size_t lengthList){
    uint8_t encodedFloatData[4*lengthList];
    size_t dataLength = encodeFloats(encodedFloatData, floatList, lengthList);
    size_t packetLength = encodePacketBare_unite(packet_buffer, deviceID, packetID, encodedFloatData, dataLength);
    return (size_t) packetLength;
}

size_t cmdJoint_encodePacket_unite(uint8_t* packet_buffer, uint8_t packetID, uint8_t deviceID, float value){
    //& NOT TESTED NOT SEEM TO WORK
    size_t length = 4;
    uint8_t data_encoded_float[4];
    encodeFloat(data_encoded_float, value);
    uint8_t tempBuffer[10];
    memset(tempBuffer, 0, 10);
    size_t totalLength = length + HEADER_SIZE;
    memcpy(tempBuffer, data_encoded_float, length);
    tempBuffer[totalLength-4] = packetID;
    tempBuffer[totalLength-3] = deviceID;
    tempBuffer[totalLength-2] = totalLength;
    tempBuffer[totalLength-1] = crc8(0xff, tempBuffer, totalLength-1);
    size_t packetLength = cobs_encode_unite(tempBuffer, totalLength, packet_buffer);
    return packetLength;
}

size_t encodeFloats(uint8_t* outputBuffer, float* floatList, size_t length){
    for(size_t i = 0; i < length; i++)
    {
        float f = floatList[i];
        uint32_t asInt = *((uint32_t*)&f);
        for (int j = 0; j < 4; j++) 
        {
		    outputBuffer[i*4 + j] = (asInt >> 8 * j) & 0xFF;
	    }
    }
    return length*sizeof(uint8_t) * 4;
}

size_t cmdJoint_encodePacket_unite2(uint8_t* packet_buffer, uint8_t packetID, uint8_t deviceID, float value){
    //NOT TESTED
    uint8_t data_encoded_float[4];
    size_t dataLength = encodeFloat(data_encoded_float, value);
    encodeFloat(data_encoded_float, value);

    struct Packet packet;
    packet.deviceID = deviceID;
    packet.packetID = packetID;
    memcpy(packet.data, data_encoded_float, dataLength);
    packet.dataLength = dataLength;

    size_t packetLength = encodePacket_unite(packet_buffer, &packet);
    return packetLength;
}

size_t decodeFloats_unite(float* outputFloatBuffer, uint8_t* inputBuffer, size_t inputBufferLength){
    size_t floatLength = inputBufferLength/4;
    uint8_t* inputBufferPointer;
    uint32_t asInt;
    for(int i = 0; i < floatLength; i++)
    {   
        asInt = 0;
        inputBufferPointer = &inputBuffer[i*4];

        for(int j = 0; j<4; j++)
        {   
            asInt = asInt | (inputBufferPointer[j] << 8 * j);
        }
        outputFloatBuffer[i] = *((float*)&asInt);
    }
    return floatLength;
}

float decodeFloat_unite(uint8_t* inputBuffer)
{
    uint32_t asInt = 0;
    for(int i = 0; i<4; i++)
    {   
        asInt = asInt | (inputBuffer[i] << 8 * i);
    }
    float f = *((float*)&asInt);
    return f;
}

int decodePacket_unite(struct Packet* packet, uint8_t* inputData, size_t inputDatalength)
{
    if (inputDatalength > MAX_PACKET_LENGTH){
        // printf("LENGTH: %d is longer than Maximum Packet Length %d\n", inputDatalength, MAX_PACKET_LENGTH);
        return -1;
    }    
    // uint8_t inputDataCopy[MAX_PACKET_LENGTH];    
    // memcpy(inputDataCopy, inputData, inputDatalength);
    size_t packetEndIndex = 0;
    while(inputData[packetEndIndex] != 0 && packetEndIndex<inputDatalength)
    {
        packetEndIndex++;
    }
    // inputDataCopy[packetEndIndex] = 0xFF;
    uint8_t packetLength = inputData[packetEndIndex-2];
    // Check if length is valid
    if(packetLength < 4 || packetLength >= inputDatalength){
        // Length is incoreect
        packet->packetID = 0;
        packet->deviceID = 0;
        return -2;
    }
    uint8_t unwrapedBuffer[MAX_PACKET_LENGTH];
    memset(unwrapedBuffer, 0, MAX_PACKET_LENGTH);
    memcpy(unwrapedBuffer, inputData, packetEndIndex);
    unwrapedBuffer[packetEndIndex] = 0xFF;
    uint8_t decodedData[MAX_PACKET_LENGTH];
    memset(decodedData, 0, MAX_PACKET_LENGTH);
    cobs_decode_unite(unwrapedBuffer, packetLength+1, decodedData);    
    uint8_t crc = decodedData[packetLength-1];
    uint8_t deviceID = decodedData[packetLength-3];
    uint8_t packetID = decodedData[packetLength-4];
    uint8_t crcCheck=crc8(0xff, decodedData, packetLength-1);
    if (crc != crcCheck){
        // CRC is not correct
        return -3;
    }
    packet->dataLength = packetLength-4;
    packet->deviceID = deviceID;
    packet->packetID = packetID;
    memset(packet->data, 0 ,MAX_PACKET_LENGTH);
    memcpy(packet->data, decodedData, packetLength-4);
    return 1; 
}

int decodePacket_unite(struct Packet* packet, const std::vector<uint8_t>& inputData, size_t inputDatalength)
{
    if (inputDatalength > MAX_PACKET_LENGTH){
        // printf("LENGTH: %d is longer than Maximum Packet Length %d\n", inputDatalength, MAX_PACKET_LENGTH);
        return -1;
    }
    // uint8_t inputDataCopy[MAX_PACKET_LENGTH];    
    // memcpy(inputDataCopy, inputData, inputDatalength);
    size_t packetEndIndex = 0;
    while(inputData[packetEndIndex] != 0 && packetEndIndex<inputDatalength)
    {
        packetEndIndex++;
    }
    // inputDataCopy[packetEndIndex] = 0xFF;
    uint8_t packetLength = inputData[packetEndIndex-2];
    // Check if length is valid
    if(packetLength < 4 || packetLength >= inputDatalength){
        // Length is incoreect
        packet->packetID = 0;
        packet->deviceID = 0;
        return -2;
    }
    uint8_t unwrapedBuffer[MAX_PACKET_LENGTH];
    memset(unwrapedBuffer, 0, MAX_PACKET_LENGTH);
    memcpy(unwrapedBuffer, inputData.data(), packetEndIndex);
    unwrapedBuffer[packetEndIndex] = 0xFF;
    uint8_t decodedData[MAX_PACKET_LENGTH];
    memset(decodedData, 0, MAX_PACKET_LENGTH);
    cobs_decode_unite(unwrapedBuffer, packetLength+1, decodedData);    
    uint8_t crc = decodedData[packetLength-1];
    uint8_t deviceID = decodedData[packetLength-3];
    uint8_t packetID = decodedData[packetLength-4];
    uint8_t crcCheck=crc8(0xff, decodedData, packetLength-1);
    if (crc != crcCheck){
        // CRC is not correct
        return -3;
    }
    packet->dataLength = packetLength-4;
    packet->deviceID = deviceID;
    packet->packetID = packetID;
    memset(packet->data, 0 ,MAX_PACKET_LENGTH);
    memcpy(packet->data, decodedData, packetLength-4);
    return 1; 
}

// Function to calculate CRC8 using polynomial 0x14D, init CRC = 0xFF, and xorOut = 0xFF
uint8_t calculate_crc8_unite(const std::vector<uint8_t>& data) {
    uint8_t crc = 0xFF; // Initial CRC value
    for (auto byte : data) {
        crc ^= byte; // XOR byte into least sig. byte of crc
        for (int i = 0; i < 8; i++) { // Loop over each bit
            if (crc & 0x80) { // If the highest bit is set
                crc = (crc << 1) ^ 0x14D; // Shift left and XOR with the polynomial
            } else {
                crc <<= 1; // Just shift left
            }
        }
    }
    // XOR the output CRC value with 0xFF
    return crc ^ 0xFF;
}


std::pair<std::vector<std::vector<uint8_t>>, std::vector<uint8_t>> BPLProtocol::packet_splitter(const std::vector<uint8_t>& buff){

            std::vector<std::vector<uint8_t>> packets;
            std::vector<uint8_t> incomplete_packet;
            size_t start = 0;

            for (size_t i = 0; i < buff.size(); ++i) {
                if (buff[i] == 0x00) {
                    packets.push_back(std::vector<uint8_t>(buff.begin() + start, buff.begin() + i));
                    start = i + 1;
                }
            }

            if (start < buff.size()) {
                incomplete_packet = std::vector<uint8_t>(buff.begin() + start, buff.end());
            }

            return {packets, incomplete_packet};
}

std::tuple<int, int, std::vector<uint8_t>, int> BPLProtocol::parse_packet(const std::vector<uint8_t>& packet_in){
    int error = 0;
    if (packet_in.size() <= 3) {
        error = -1 ;
        return std::make_tuple(0, 0, std::vector<uint8_t>{}, error);
    }
    try {
        std::vector<uint8_t> decoded_packet;                
        uint8_t size_decoded = cobs_decode_unite(packet_in, decoded_packet);
        if (decoded_packet[decoded_packet.size() - 2] != decoded_packet.size()) {
            // Incorrect length
            error = -2;
            return std::make_tuple(0, 0, std::vector<uint8_t>{}, error);
        } else if (calculate_crc8_unite(std::vector<uint8_t>(decoded_packet.begin(), decoded_packet.end() - 1)) == decoded_packet.back()) {
            int device_id = decoded_packet[decoded_packet.size() - 3];
            int packet_id = decoded_packet[decoded_packet.size() - 4];
            std::vector<uint8_t> rx_data(decoded_packet.begin(), decoded_packet.end() - 4);
            error = 1;
            return std::make_tuple(device_id, packet_id, rx_data, error);
        } else {
            // CRC error
            error = -3;
            return std::make_tuple(0, 0, std::vector<uint8_t>{}, error);
        }
    } catch (const std::exception& e) {
        // COBS decoding error
        error = -4; 
        return std::make_tuple(0, 0, std::vector<uint8_t>{}, error);
    }
}

std::vector<std::tuple<int, int, std::vector<uint8_t>, int>> PacketReader::receive_bytes(const std::vector<uint8_t>& data) {
        std::vector<std::tuple<int, int, std::vector<uint8_t>, int>> packet_list;
        //auto [encoded_packets, remainder] = BPLProtocol::packet_splitter(std::vector<uint8_t>(incomplete_packets.begin(), incomplete_packets.end()) + data);
        auto [encoded_packets, remainder] = BPLProtocol::packet_splitter(data);
        incomplete_packets = remainder;
        for (const auto& encoded_packet : encoded_packets) {
            if (encoded_packet.empty()) continue;
            auto decoded_packet = BPLProtocol::parse_packet(encoded_packet);
            packet_list.push_back(decoded_packet);
        }
        return packet_list;
}

std::vector<Packet> PacketReader::receive_bytes_decode(const std::vector<uint8_t>& data) {
        std::vector<Packet> packet_list;
        //auto [encoded_packets, remainder] = BPLProtocol::packet_splitter(std::vector<uint8_t>(incomplete_packets.begin(), incomplete_packets.end()) + data);
        auto [encoded_packets, remainder] = BPLProtocol::packet_splitter(data);
        incomplete_packets = remainder;
        for (const auto& encoded_packet : encoded_packets) {
            if (encoded_packet.empty()) continue;
            Packet packet_decoded;
            int result =  decodePacket_unite(&packet_decoded, encoded_packet, encoded_packet.size());
            if (result < 0) continue;
            packet_list.push_back(packet_decoded);
        }
        return packet_list;
}