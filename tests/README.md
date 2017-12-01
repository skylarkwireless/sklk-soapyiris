# Some simple tests for the Iris

## Data format test

Uses SoapySDR python bindings to test various data format options
both for streaming and over the wire.

```
python DataFormatsTest.py

#or for a specific device by serial
IRIS=XXXX python DataFormatsTest.py

```

## Full duplex traffic

A C++ based example that transmits (empty) packets with timestamps
based on the timestamps of incoming received packets. The demo
duplicates the network traffic patterns that would be seen
in a similar real-time streaming application and keeps track of
stream errors such as overflows, underflows, and time errors. 

```
cd build-directory
#Usage: tests/IrisFullDuplex [argsString] [rate] [numCh]
tests/IrisFullDuplex serial=XXXX 7.68e6 1

```
