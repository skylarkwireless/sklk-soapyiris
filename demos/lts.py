#!/usr/bin/python
#
#	Library for generating and performing LTS-related operations
#
#	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
#	INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
#	FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
#	DEALINGS IN THE SOFTWARE.
#
#	(c) 2016 info@skylarkwireless.com 

import numpy as np

lts_freq = np.array([0,0,0,0,0,0,1,1,-1,-1,1,1,-1,1,-1,1,1,1,1,1,1,-1,-1,1,1,-1,1,-1,1,1,1,1,0,1,-1,-1,1,1,-1,1,-1,1,-1,-1,-1,-1,-1,1,1,-1,-1,1,-1,1,-1,1,1,1,1,0,0,0,0,0])

def genLTS(upsample=1, cp=32):
	'''Generate a time-domain 802.11 LTS with a cyclic prefix of "cp" (32) and upsampled by a factor of "up" (1).'''
	up_zeros = np.zeros(len(lts_freq)//2*(upsample-1))
	lts_freq_up = np.concatenate((up_zeros,lts_freq,up_zeros))
	signal = np.fft.ifft(np.fft.ifftshift(lts_freq_up))
	signal = signal/np.absolute(signal).max()  #normalize
	
	#Now affix the cyclic prefix
	signal = np.concatenate((signal[len(signal) - cp:], signal, signal))  #could use tile...

	return signal

def findLTS(iq, thresh=600, us=1):
	'''
		Find the indices of all LTSs in the input "iq" signal, upsampled by a factor of "up".
		"thresh" (600) sets sensitivity.
		
		Returns: best (highest LTS peak), actual_ltss (the list of all detected LTSs), and peaks (the correlated signal, multiplied by itself delayed by 1/2 an LTS)
	'''
	
	gold = genLTS(upsample=us, cp=0)[:64*us]
	cored = np.correlate(iq,gold,'full')
	peaks = np.concatenate((np.zeros(64*us),cored)) * np.concatenate((np.conj(cored),np.zeros(64*us)))
	t = np.mean(np.abs(iq))*thresh
	ltss = np.where(peaks > t)[0]
	actual_ltss = []
	for l in ltss:
		if not peaks[l+us*64] > peaks[l]:  #if there is another peak 64 samples in the future, this was probably a false positive from the CP
			actual_ltss.append(l - us*128) #return the start of the LTS, not the end.
	best = np.argmax(peaks) - us*128
	return best, actual_ltss, peaks

def getChanEst(iq, us=1):
	'''Takes an "iq" stream of 128*"us" length and computes the channel estimates.'''
	iq = iq[::us] #downsample
	return np.mean(np.fft.fftshift(np.fft.fft(np.reshape(iq, (2,64) ),axis=1),axes=(1)),axis=0)*lts_freq #multiply and divide are the same since they're all 1 or -1

def getCFO(iq,us=1):
	'''Takes an "iq" stream of 128*"us" length and computes the CFO.'''
	ltss = np.reshape(iq, (us*64,2) )
	return np.mean(np.angle(ltss[:,0]*np.conj(ltss[:,1])))/64