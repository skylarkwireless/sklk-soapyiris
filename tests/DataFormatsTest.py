## Copyright (c) 2017 Skylark Wireless LLC
## SPDX-License-Identifier: BSD-3-Clause

########################################################################
## Test streaming API + framer/deframer with internal loopback
########################################################################

import SoapySDR
from SoapySDR import * #SOAPY_SDR_ constants
import numpy as np
import time
import os
import math
import unittest

SDR_ARGS = dict(driver="iris")
if "IRIS" in os.environ: SDR_ARGS["serial"] = os.environ["IRIS"]
RATE = 20e6

class TestDataFormats(unittest.TestCase):

    def setUp(self):
        self.sdr = SoapySDR.Device(SDR_ARGS)

        #some default sample rates
        self.sdr.setSampleRate(SOAPY_SDR_RX, 0, RATE)
        self.sdr.setSampleRate(SOAPY_SDR_RX, 1, RATE)
        self.sdr.setSampleRate(SOAPY_SDR_TX, 0, RATE)
        self.sdr.setSampleRate(SOAPY_SDR_TX, 1, RATE)

        self.sdr.writeSetting("FPGA_DIQ_MODE", "LOOPB") #tx deframer -> rx framer

    def tearDown(self):
        self.sdr = None

    def genTestWaveform(self, numSamps, npType, shift=0, mod=(1<<15)):
        waveTxA = np.array([[0, 0]]*numSamps, npType)
        waveTxB = np.array([[0, 0]]*numSamps, npType)
        for i in range(numSamps):
            waveTxA[i][0] = ((i << 2) | 0) % mod
            waveTxA[i][1] = ((i << 2) | 1) % mod
            waveTxB[i][0] = ((i << 2) | 2) % mod
            waveTxB[i][1] = ((i << 2) | 3) % mod
        return waveTxA << shift, waveTxB << shift

    def loopbackWaves(self, numSamps, rxFormat, rxChans, rxType, rxWire, txFormat, txChans, txType, txWire, shift=0, mod=(1<<15)):
        print("#"*40)
        print("## Num samples: %d"%numSamps)
        print("## Rx config: %s%s"%(rxFormat, rxChans))
        print("## Tx config: %s%s"%(txFormat, txChans))
        print("#"*40)
        print("")

        #create streams
        rxStream = self.sdr.setupStream(SOAPY_SDR_RX, rxFormat, rxChans, dict(WIRE=rxWire))
        txStream = self.sdr.setupStream(SOAPY_SDR_TX, txFormat, txChans, dict(WIRE=txWire))

        #generate a waveform
        waveTxA, waveTxB = self.genTestWaveform(numSamps, txType, shift=shift, mod=mod)
        waveRxA = np.array([[0, 0]]*numSamps, rxType)
        waveRxB = np.array([[0, 0]]*numSamps, rxType)

        #transmit and receive at this time in the future
        flags = SOAPY_SDR_WAIT_TRIGGER | SOAPY_SDR_END_BURST
        self.sdr.activateStream(txStream)
        sr = self.sdr.writeStream(txStream, [waveTxA, waveTxB], numSamps, flags)
        self.assertEqual(sr.ret, numSamps)
        print(waveTxA[:20])
        print(waveTxA[-20:])

        #receive a waveform at the same time
        self.sdr.activateStream(rxStream, flags, 0, numSamps)

        #trigger in the near future
        time.sleep(0.1)
        self.sdr.writeSetting("TRIGGER_GEN", "")

        sr = self.sdr.readStream(rxStream, [waveRxA, waveRxB], numSamps, timeoutUs=int(1e6))
        self.assertEqual(sr.ret, numSamps)

        print(waveRxA[:20])
        print(waveRxA[-20:])

        #look at any async messages
        sr = self.sdr.readStreamStatus(txStream, timeoutUs=int(1e6))
        print(sr)
        self.assertEqual(sr.ret, 0)
        self.assertNotEqual(sr.flags & SOAPY_SDR_END_BURST, 0)

        delay = 1
        np.testing.assert_array_equal(waveRxA[delay:], waveTxA[:-delay])
        if len(rxChans) == 2:
            np.testing.assert_array_equal(waveRxB[delay:], waveTxB[:-delay])

        self.sdr.deactivateStream(rxStream)
        self.sdr.deactivateStream(txStream)
        self.sdr.closeStream(rxStream)
        self.sdr.closeStream(txStream)

    def testLoopbackWaves(self):
        for numSamps in (0, 100, 500):
            for extra in range(1, 8):
                self.loopbackWaves(numSamps=numSamps+extra, shift=4,
                    rxFormat=SOAPY_SDR_CS16, rxChans=[0, 1], rxType=np.int16, rxWire='',
                    txFormat=SOAPY_SDR_CS16, txChans=[0, 1], txType=np.int16, txWire='')
                self.loopbackWaves(numSamps=numSamps+extra, shift=4,
                    rxFormat=SOAPY_SDR_CS16, rxChans=[0], rxType=np.int16, rxWire='',
                    txFormat=SOAPY_SDR_CS16, txChans=[0], txType=np.int16, txWire='')
                self.loopbackWaves(numSamps=numSamps+extra, mod=128,
                    rxFormat=SOAPY_SDR_CS16, rxChans=[0], rxType=np.uint16, rxWire=SOAPY_SDR_CS8,
                    txFormat=SOAPY_SDR_CS16, txChans=[0], txType=np.uint16, txWire=SOAPY_SDR_CS8)
                self.loopbackWaves(numSamps=numSamps+extra, mod=128,
                    rxFormat=SOAPY_SDR_CS16, rxChans=[0, 1], rxType=np.uint16, rxWire=SOAPY_SDR_CS8,
                    txFormat=SOAPY_SDR_CS16, txChans=[0, 1], txType=np.uint16, txWire=SOAPY_SDR_CS8)

    def loopbackBytes(self, numSamps, rxFormat, rxChans, rxWire, txFormat, txChans, txWire):
        print("#"*40)
        print("## Num samples: %d"%numSamps)
        print("## Rx config: %s%s"%(rxFormat, rxChans))
        print("## Tx config: %s%s"%(txFormat, txChans))
        print("#"*40)
        print("")

        rxSize = SoapySDR.formatToSize(rxFormat)
        txSize = SoapySDR.formatToSize(txFormat)

        #create streams
        rxStream = self.sdr.setupStream(SOAPY_SDR_RX, rxFormat, rxChans, dict(WIRE=rxWire))
        txStream = self.sdr.setupStream(SOAPY_SDR_TX, txFormat, txChans, dict(WIRE=txWire))

        #generate a waveform
        waveTxA = np.fromstring(np.random.bytes(txSize*numSamps), np.uint8)
        waveTxB = np.fromstring(np.random.bytes(txSize*numSamps), np.uint8)
        waveRxA = np.array([0]*rxSize*numSamps, np.uint8)
        waveRxB = np.array([0]*rxSize*numSamps, np.uint8)

        #transmit and receive at this time in the future
        flags = SOAPY_SDR_WAIT_TRIGGER | SOAPY_SDR_END_BURST
        self.sdr.activateStream(txStream)
        sr = self.sdr.writeStream(txStream, [waveTxA, waveTxB], numSamps, flags)
        self.assertEqual(sr.ret, numSamps)

        #receive a waveform at the same time
        self.sdr.activateStream(rxStream, flags, 0, numSamps)

        #trigger in the near future
        time.sleep(0.1)
        self.sdr.writeSetting("TRIGGER_GEN", "")

        sr = self.sdr.readStream(rxStream, [waveRxA, waveRxB], numSamps, timeoutUs=int(1e6))
        self.assertEqual(sr.ret, numSamps)

        #look at any async messages
        sr = self.sdr.readStreamStatus(txStream, timeoutUs=int(1e6))
        print(sr)
        self.assertEqual(sr.ret, 0)
        self.assertNotEqual(sr.flags & SOAPY_SDR_END_BURST, 0)

        delay = txSize
        np.testing.assert_array_equal(waveRxA[delay:], waveTxA[:-delay])
        if len(rxChans) == 2:
            np.testing.assert_array_equal(waveRxB[delay:], waveTxB[:-delay])

        self.sdr.deactivateStream(rxStream)
        self.sdr.deactivateStream(txStream)
        self.sdr.closeStream(rxStream)
        self.sdr.closeStream(txStream)

    def testLoopbackBytes(self):
        for numSamps in (0, 100, 500):
            for extra in range(1, 8):
                self.loopbackBytes(numSamps=numSamps+extra,
                    rxFormat=SOAPY_SDR_CS16, rxChans=[0, 1], rxWire=SOAPY_SDR_CS16,
                    txFormat=SOAPY_SDR_CS16, txChans=[0, 1], txWire=SOAPY_SDR_CS16)
                self.loopbackBytes(numSamps=numSamps+extra,
                    rxFormat=SOAPY_SDR_CS16, rxChans=[0], rxWire=SOAPY_SDR_CS16,
                    txFormat=SOAPY_SDR_CS16, txChans=[0], txWire=SOAPY_SDR_CS16)
                self.loopbackBytes(numSamps=numSamps+extra,
                    rxFormat=SOAPY_SDR_CS12, rxChans=[0, 1], rxWire='',
                    txFormat=SOAPY_SDR_CS12, txChans=[0, 1], txWire='')
                self.loopbackBytes(numSamps=numSamps+extra,
                    rxFormat=SOAPY_SDR_CS12, rxChans=[0], rxWire='',
                    txFormat=SOAPY_SDR_CS12, txChans=[0], txWire='')
                self.loopbackBytes(numSamps=numSamps+extra,
                    rxFormat=SOAPY_SDR_CS8, rxChans=[0], rxWire='',
                    txFormat=SOAPY_SDR_CS8, txChans=[0], txWire='')
                self.loopbackBytes(numSamps=numSamps+extra,
                    rxFormat=SOAPY_SDR_CS8, rxChans=[0, 1], rxWire='',
                    txFormat=SOAPY_SDR_CS8, txChans=[0, 1], txWire='')

if __name__ == '__main__':
    unittest.main()
