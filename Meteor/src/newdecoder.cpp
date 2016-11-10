//============================================================================
// Name        : Meteor LRPT Decoder
// Author      : Lucas Teske
// Version     : 1.0
// Copyright   : Copyright 2016
// Description : Meteor LRPT Decoder
//============================================================================

#include <iostream>
#include <memory.h>
#include <cstdint>
#include <sathelper.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include "Display.h"
#include "ChannelWriter.h"

using namespace std;

#define DUMP_CORRUPTED_PACKETS

#define FRAMESIZE 1024
#define FRAMEBITS (FRAMESIZE * 8)
#define PARITY_OFFSET 892
#define CODEDFRAMESIZE (FRAMEBITS * 2)
#define MINCORRELATIONBITS 46
#define RSBLOCKS 4
#define RSPARITYSIZE 32
#define RSPARITYBLOCK (RSPARITYSIZE * RSBLOCKS)
#define SYNCWORDSIZE 32
#define TIMEOUT 2

const uint64_t UW0 = 0xfca2b63db00d9794;
const uint64_t UW1 = 0x56fbd394daa4c1c2;
const uint64_t UW2 = 0x035d49c24ff2686b;
const uint64_t UW3 = 0xa9042c6b255b3e3d;

int main() {
    uint8_t codedData[CODEDFRAMESIZE];
    uint8_t decodedData[FRAMESIZE];
    uint8_t rsCorrectedData[FRAMESIZE];
    uint8_t rsWorkBuffer[255];

    uint64_t droppedPackets = 0;
    uint64_t averageRSCorrections = 0;
    uint64_t averageVitCorrections = 0;
    uint64_t frameCount = 0;
    uint64_t lostPackets = 0;
    int64_t lostPacketsPerFrame[256];
    int64_t lastPacketCount[256];
    int64_t receivedPacketsPerFrame[256];
    uint32_t checkTime = 0;

    SatHelper::Correlator correlator;
    SatHelper::PacketFixer packetFixer;
    SatHelper::Viterbi27 viterbi(FRAMEBITS);
    SatHelper::ReedSolomon reedSolomon(PARITY_OFFSET);
    SatHelper::DeRandomizer deRandomizer;
    ChannelWriter channelWriter("channels");

    Display display;

    for (int i=0; i<256;i++) {
      lostPacketsPerFrame[i] = 0;
      lastPacketCount[i] = -1;
      receivedPacketsPerFrame[i] = -1;
    }

    correlator.addWord(UW0);
    correlator.addWord(UW1);
    correlator.addWord(UW2);
    correlator.addWord(UW3);

    // Socket Init
    SatHelper::TcpServer tcpServer;
    tcpServer.Listen(5000);
    cout << "Waiting for a client connection" << endl;

    SatHelper::TcpSocket client = tcpServer.Accept();
    cout << "Client connected!" << endl;

    SatHelper::ScreenManager::Clear();

    while (true) {
        uint32_t chunkSize = CODEDFRAMESIZE;
        try {
            checkTime = SatHelper::Tools::getTimestamp();
            while (client.AvailableData() < CODEDFRAMESIZE) {
                if (SatHelper::Tools::getTimestamp() - checkTime > TIMEOUT) {
                    throw SatHelper::ClientDisconnectedException();
                }
            }

            client.Receive((char *) codedData, chunkSize);
            correlator.correlate(codedData, chunkSize);

            uint32_t word = correlator.getCorrelationWordNumber();
            uint32_t pos = correlator.getHighestCorrelationPosition();
            uint32_t corr = correlator.getHighestCorrelation();
            SatHelper::PhaseShift phaseShift;
            switch (word) {
				case 0: phaseShift = SatHelper::PhaseShift::DEG_0; break;
				case 1: phaseShift = SatHelper::PhaseShift::DEG_90; break;
				case 2: phaseShift = SatHelper::PhaseShift::DEG_180; break;
				case 3: phaseShift = SatHelper::PhaseShift::DEG_270; break;
            }

            if (corr < MINCORRELATIONBITS) {
                cerr << "Correlation didn't match criteria of " << MINCORRELATIONBITS << " bits." << std::endl;
                continue;
            }

            // Sync Frame
            if (pos != 0) {
                // Shift position
                char *shiftedPosition = (char *) codedData + pos;
                memcpy(codedData, shiftedPosition, CODEDFRAMESIZE - pos); // Copy from p to chunk size to start of codedData
                chunkSize = pos; // Read needed bytes to fill a frame.
                checkTime = SatHelper::Tools::getTimestamp();
                while (client.AvailableData() < chunkSize) {
                    if (SatHelper::Tools::getTimestamp() - checkTime > TIMEOUT) {
                        throw SatHelper::ClientDisconnectedException();
                    }
                }

                client.Receive((char *) (codedData + CODEDFRAMESIZE - pos), chunkSize);
            }

            // Fix Phase Shift
            packetFixer.fixPacket(codedData, CODEDFRAMESIZE, phaseShift, false);

            // Viterbi Decode
            viterbi.decode(codedData, decodedData);
            float signalErrors = viterbi.GetPercentBER(); // 0 to 16
            signalErrors = 100 - (signalErrors * 10);
            uint8_t signalQuality = signalErrors < 0 ? 0 : (uint8_t)signalErrors;
            // DeRandomize Stream
            uint8_t skipsize = (SYNCWORDSIZE/8);
            memcpy(decodedData, decodedData + skipsize, FRAMESIZE-skipsize);
            deRandomizer.DeRandomize(decodedData, CODEDFRAMESIZE);

            averageVitCorrections += viterbi.GetBER();
            frameCount++;

            // Reed Solomon Error Correction
            int32_t derrors[4] = { 0, 0, 0, 0 };
            for (int i=0; i<RSBLOCKS; i++) {
              reedSolomon.deinterleave(decodedData, rsWorkBuffer, i, RSBLOCKS);
              derrors[i] = reedSolomon.decode_rs8(rsWorkBuffer);
              reedSolomon.interleave(rsWorkBuffer, rsCorrectedData, i, RSBLOCKS);
            }

            if (derrors[0] == -1 && derrors[1] == -1 && derrors[2] == -1 && derrors[3] == -1) {
              droppedPackets++;
              #ifdef DUMP_CORRUPTED_PACKETS
              channelWriter.dumpCorruptedPacket(codedData, FRAMESIZE, 0);
              channelWriter.dumpCorruptedPacket(decodedData, FRAMESIZE, 1);
              channelWriter.dumpCorruptedPacket(rsCorrectedData, FRAMESIZE, 2);
              channelWriter.dumpCorruptedPacketStatistics(viterbi.GetBER(), corr);
              #endif
              uint16_t partialVitCorrections = (uint16_t) (averageVitCorrections / frameCount);
              uint8_t partialRSCorrections = (uint8_t) (averageRSCorrections / frameCount);
              display.update(0, 0, 0, viterbi.GetBER(), FRAMEBITS, derrors,
                      signalQuality, corr, 0,
                      lostPackets, partialVitCorrections, partialRSCorrections,
                      droppedPackets, receivedPacketsPerFrame, lostPacketsPerFrame, frameCount);

              display.show();
              continue;
            } else {
              averageRSCorrections += derrors[0] != -1 ? derrors[0] : 0;
              averageRSCorrections += derrors[1] != -1 ? derrors[1] : 0;
              averageRSCorrections += derrors[2] != -1 ? derrors[2] : 0;
              averageRSCorrections += derrors[3] != -1 ? derrors[3] : 0;
            }

            // Packet Header Filtering
            //uint8_t versionNumber = (*rsCorrectedData) & 0xC0 >> 6;
            uint8_t scid = ((*rsCorrectedData) & 0x3F) << 2 | (*(rsCorrectedData+1) & 0xC0) >> 6;
            uint8_t vcid = (*(rsCorrectedData+1)) & 0x3F;

            // Packet Counter from Packet
            uint32_t counter = *((uint32_t *) (rsCorrectedData+2));
            counter = SatHelper::Tools::swapEndianess(counter);
            counter &= 0xFFFFFF00;
            counter = counter >> 8;
            channelWriter.writeChannel(rsCorrectedData, FRAMESIZE - RSPARITYBLOCK - (SYNCWORDSIZE/8), vcid);

            int lostCount = counter - lastPacketCount[vcid] - 1;
            if (lastPacketCount[vcid]+1 != counter && lastPacketCount[vcid] > -1 && lostCount > 0) {
              int lostCount = counter - lastPacketCount[vcid] - 1;
              lostPackets += lostCount;
              lostPacketsPerFrame[vcid] += lostCount;
            }

            lastPacketCount[vcid] = counter;
            receivedPacketsPerFrame[vcid] = receivedPacketsPerFrame[vcid] == -1 ? 1 : receivedPacketsPerFrame[vcid] + 1;
            uint16_t phaseCorr;
            switch (phaseShift) {
				case SatHelper::PhaseShift::DEG_0: phaseCorr = 0; break;
				case SatHelper::PhaseShift::DEG_90: phaseCorr = 90; break;
				case SatHelper::PhaseShift::DEG_180: phaseCorr = 180; break;
				case SatHelper::PhaseShift::DEG_270: phaseCorr = 270; break;
            }
            uint16_t partialVitCorrections = (uint16_t) (averageVitCorrections / frameCount);
            uint8_t partialRSCorrections = (uint8_t) (averageRSCorrections / frameCount);

            display.update(scid, vcid, (uint64_t) counter, (int16_t) viterbi.GetBER(), FRAMEBITS, derrors,
                    signalQuality, corr, phaseCorr,
                    lostPackets, partialVitCorrections, partialRSCorrections,
                    droppedPackets, receivedPacketsPerFrame, lostPacketsPerFrame, frameCount);

            display.show();

        } catch (SatHelper::SocketException &e) {
            cerr << endl;
            cerr << "Client disconnected" << endl;
            cerr << "   " << e.what() << endl;
            break;
        }
    }

    client.Close();
    return 0;
}
