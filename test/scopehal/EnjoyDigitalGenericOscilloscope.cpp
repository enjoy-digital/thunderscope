/***********************************************************************************************************************
*                                                                                                                      *
* libscopehal v0.1                                                                                                     *
*                                                                                                                      *
* Copyright (c) 2012-2021 Andrew D. Zonenberg and contributors                                                         *
* Copyright (c) 2022      Florent Kermarrec                                                                            *
* All rights reserved.                                                                                                 *
*                                                                                                                      *
* Redistribution and use in source and binary forms, with or without modification, are permitted provided that the     *
* following conditions are met:                                                                                        *
*                                                                                                                      *
*    * Redistributions of source code must retain the above copyright notice, this list of conditions, and the         *
*      following disclaimer.                                                                                           *
*                                                                                                                      *
*    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the       *
*      following disclaimer in the documentation and/or other materials provided with the distribution.                *
*                                                                                                                      *
*    * Neither the name of the author nor the names of any contributors may be used to endorse or promote products     *
*      derived from this software without specific prior written permission.                                           *
*                                                                                                                      *
* THIS SOFTWARE IS PROVIDED BY THE AUTHORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   *
* TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL *
* THE AUTHORS BE HELD LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES        *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR       *
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE       *
* POSSIBILITY OF SUCH DAMAGE.                                                                                          *
*                                                                                                                      *
***********************************************************************************************************************/

#include "scopehal.h"
#include "EnjoyDigitalGenericOscilloscope.h"
#include "SCPISocketTransport.h"
#include "EdgeTrigger.h"

using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

EnjoyDigitalGenericOscilloscope::EnjoyDigitalGenericOscilloscope(SCPITransport* transport)
	: SCPIOscilloscope(transport)
	/*
	, m_triggerArmed(false)
	, m_triggerOneShot(false)*/
{
	//Create the data-plane socket for our waveforms
	auto socktrans = dynamic_cast<SCPISocketTransport*>(transport);
	if(!socktrans)
		LogFatal("Enjoy-Digital Generic Oscilloscope only support SCPISocketTransport\n");
	m_waveformTransport = new SCPISocketTransport(socktrans->GetHostname() + ":50101");

	//Last digit of the model number is the number of channels
	fprintf(stderr, "%s", m_model.c_str());
	int model_number = atoi(m_model.c_str() + 3);	//FIXME: are all series IDs 3 chars e.g. "RTM"?
	int nchans = model_number % 10;

	for(int i=0; i<nchans; i++)
	{
		// Hardware name of the channel
		string chname = string("C0");
		chname[1] += i;

		// Color the channels (based on Antikernel Labs's color sequence).
		string color = "#ffffff";
		switch(i)
		{
			case 0:
				color = "#ffff80";
				break;

			case 1:
				color = "#ff8080";
				break;

			case 2:
				color = "#80ffff";
				break;

			case 3:
				color = "#80ff80";
				break;

			// TODO: colors for the other 4 channels
		}

		// Create the channel
		auto chan = new OscilloscopeChannel(
			this,
			chname,
			OscilloscopeChannel::CHANNEL_TYPE_ANALOG,
			color,
			1,
			i,
			true);
		m_channels.push_back(chan);
		chan->SetDefaultDisplayName();
	}

	m_analogChannelCount = nchans;
}

EnjoyDigitalGenericOscilloscope::~EnjoyDigitalGenericOscilloscope()
{
	delete m_waveformTransport;
	m_waveformTransport = NULL;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Accessors

string EnjoyDigitalGenericOscilloscope::GetDriverNameInternal()
{
	return "enjoy-digital";
}

unsigned int EnjoyDigitalGenericOscilloscope::GetInstrumentTypes()
{
	return Instrument::INST_OSCILLOSCOPE;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Device interface functions

void EnjoyDigitalGenericOscilloscope::FlushConfigCache()
{
	lock_guard<recursive_mutex> lock(m_cacheMutex);

	m_channelOffsets.clear();
	m_channelVoltageRanges.clear();
	//m_channelsEnabled.clear();
}

bool EnjoyDigitalGenericOscilloscope::IsChannelEnabled(size_t /*i*/)
{
	// TODO; for now declared channels are always enabled.
	return true;
}

void EnjoyDigitalGenericOscilloscope::EnableChannel(size_t /*i*/)
{
	// TODO; for now declared channels are always enabled.
}

void EnjoyDigitalGenericOscilloscope::DisableChannel(size_t /*i*/)
{
	// TODO; for now declared channels are always enabled.
}

vector<OscilloscopeChannel::CouplingType> EnjoyDigitalGenericOscilloscope::GetAvailableCouplings(size_t /*i*/)
{
	// TODO; for now always use COUPLE_DC_50.
	vector<OscilloscopeChannel::CouplingType> ret;
	ret.push_back(OscilloscopeChannel::COUPLE_DC_50);
	return ret;
}

OscilloscopeChannel::CouplingType EnjoyDigitalGenericOscilloscope::GetChannelCoupling(size_t /*i*/)
{
	// TODO; for now always use COUPLE_DC_50.
	return OscilloscopeChannel::COUPLE_DC_50;
}

void EnjoyDigitalGenericOscilloscope::SetChannelCoupling(size_t /*i*/, OscilloscopeChannel::CouplingType /*type*/)
{
	// TODO; for now always use COUPLE_DC_50.
}

double EnjoyDigitalGenericOscilloscope::GetChannelAttenuation(size_t /*i*/)
{
	// TODO; for now attenuation = 1.
	return 1;
}

void EnjoyDigitalGenericOscilloscope::SetChannelAttenuation(size_t /*i*/, double /*atten*/)
{
	// TODO; for now attenuation = 1.
}

int EnjoyDigitalGenericOscilloscope::GetChannelBandwidthLimit(size_t /*i*/)
{
	// TODO; for no bandwidth limit.
	return 0;
}

void EnjoyDigitalGenericOscilloscope::SetChannelBandwidthLimit(size_t /*i*/, unsigned int /*limit_mhz*/)
{
	// TODO; for no bandwidth limit.
}

float EnjoyDigitalGenericOscilloscope::GetChannelVoltageRange(size_t i, size_t /*stream*/)
{
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		if(m_channelVoltageRanges.find(i) != m_channelVoltageRanges.end())
			return m_channelVoltageRanges[i];
	}

	lock_guard<recursive_mutex> lock2(m_mutex);

	/* Get GAIN (dB). */
	int db;
	m_transport->SendCommand(m_channels[i]->GetHwname() + ":GAIN?");
	string reply = m_transport->ReadReply();
	sscanf(reply.c_str(), "%d", &db);

	/* Convert Gain (dB) to VFS. */
	float frac_gain = pow(10, db / 20.0f);
	float vfs = 2.0 / frac_gain;

	LogDebug("Channel gain is %d dB (%.2f V/V, Vfs = %.3f)\n", db, frac_gain, vfs);

	lock_guard<recursive_mutex> lock(m_cacheMutex);
	m_channelVoltageRanges[i] = vfs;
	return vfs;
}

void EnjoyDigitalGenericOscilloscope::SetChannelVoltageRange(size_t i, size_t /*stream*/, float range)
{
	// TODO; for now only reported by oscilloscope.
	lock_guard<recursive_mutex> lock(m_cacheMutex);
	m_channelVoltageRanges[i] = range;
}

OscilloscopeChannel* EnjoyDigitalGenericOscilloscope::GetExternalTrigger()
{
	// TODO; for now no external trigger.
	return NULL;
}

float EnjoyDigitalGenericOscilloscope::GetChannelOffset(size_t i, size_t /*stream*/)
{
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);

		if(m_channelOffsets.find(i) != m_channelOffsets.end())
			return m_channelOffsets[i];
	}

	lock_guard<recursive_mutex> lock2(m_mutex);

	m_transport->SendCommand(m_channels[i]->GetHwname() + ":OFFS?");

	string reply = m_transport->ReadReply();
	float offset;
	sscanf(reply.c_str(), "%f", &offset);
	lock_guard<recursive_mutex> lock(m_cacheMutex);
	m_channelOffsets[i] = offset;
	return offset;
}

void EnjoyDigitalGenericOscilloscope::SetChannelOffset(size_t i, size_t /*stream*/, float offset)
{
	// TODO; for now only reported by oscilloscope.
	lock_guard<recursive_mutex> lock2(m_cacheMutex);
	m_channelOffsets[i] = offset;
}

Oscilloscope::TriggerMode EnjoyDigitalGenericOscilloscope::PollTrigger()
{
	// TODO; for now always report triggered.
	return TRIGGER_MODE_TRIGGERED;
}

bool EnjoyDigitalGenericOscilloscope::AcquireData()
{
	const int depth_max = 1000000;  // FIXME: Allow configuration.
	static uint8_t waveform[depth_max];

	lock_guard<recursive_mutex> lock2(m_mutex);

	string reply;

	/* Get Waveform Data. */
	int depth;
	m_transport->SendCommand("SAMP:DEPTH?");
	reply = m_transport->ReadReply();
	sscanf(reply.c_str(), "%d", &depth);
	m_waveformTransport->ReadRawData(depth, waveform);

	/* Get Waveform Timescale. */
	int timescale;
	m_transport->SendCommand("SAMP:TIM?");
	reply = m_transport->ReadReply();
	sscanf(reply.c_str(), "%d", &timescale);

	/* Prepare Waveform. */
	map<int, vector<AnalogWaveform*> > pending_waveforms;
	for(size_t j=0; j<m_analogChannelCount; j++) {

		/* Set Waveform Timescale/Timestamp. */
		AnalogWaveform* cap = new AnalogWaveform;
		double t                 = GetTime();
		cap->m_timescale         = timescale;
		cap->m_triggerPhase      = 0;
		cap->m_startTimestamp    = floor(t);
		cap->m_startFemtoseconds = (t - cap->m_startTimestamp) * FS_PER_SECOND;

		/* Re-Scale Samples and add them to the Waveform */
		float fullscale = GetChannelVoltageRange(0, 0);
		float scale     = fullscale / 256.0f;
		float offset    = GetChannelOffset(0, 0);
		cap->Resize(depth/m_analogChannelCount);
		for(size_t i=0; i<depth/m_analogChannelCount; i++)
		{
			cap->m_offsets[i]   = i;
			cap->m_durations[i] = 1;
			cap->m_samples[i]   = ((waveform[m_analogChannelCount*j+i] - 128.0f) * scale) + offset;
		}

		/* Done, Push Waveform */
		lock_guard<recursive_mutex> lock(m_mutex);
		pending_waveforms[j].push_back(cap);
	}

	/* Now that we have all of the pending waveforms, save them in sets across all channels. */
	SequenceSet s;
	for(size_t j=0; j<m_analogChannelCount; j++)
	{
		if(IsChannelEnabled(j))
			s[m_channels[j]] = pending_waveforms[j][0];
	}
	m_pendingWaveforms.push_back(s);

	return true;
}

void EnjoyDigitalGenericOscilloscope::Start()
{
	// TODO; for now always running.
}

void EnjoyDigitalGenericOscilloscope::StartSingleTrigger()
{
	// TODO; for now always running.
}

void EnjoyDigitalGenericOscilloscope::Stop()
{
	// TODO; for now always running.
}

void EnjoyDigitalGenericOscilloscope::ForceTrigger()
{
	// TODO; for now always running.
}

bool EnjoyDigitalGenericOscilloscope::IsTriggerArmed()
{
	// TODO; for now always running.
	return true;
}

vector<uint64_t> EnjoyDigitalGenericOscilloscope::GetSampleRatesNonInterleaved()
{
	// FIXME
	vector<uint64_t> ret;
	return ret;
}

vector<uint64_t> EnjoyDigitalGenericOscilloscope::GetSampleRatesInterleaved()
{
	// FIXME
	vector<uint64_t> ret;
	return ret;
}

set<Oscilloscope::InterleaveConflict> EnjoyDigitalGenericOscilloscope::GetInterleaveConflicts()
{
	// FIXME
	set<Oscilloscope::InterleaveConflict> ret;
	return ret;
}

vector<uint64_t> EnjoyDigitalGenericOscilloscope::GetSampleDepthsNonInterleaved()
{
	// FIXME
	vector<uint64_t> ret;
	return ret;
}

vector<uint64_t> EnjoyDigitalGenericOscilloscope::GetSampleDepthsInterleaved()
{
	// FIXME
	vector<uint64_t> ret;
	return ret;
}

uint64_t EnjoyDigitalGenericOscilloscope::GetSampleRate()
{
	// TODO.
	return 625000000L;
}

uint64_t EnjoyDigitalGenericOscilloscope::GetSampleDepth()
{
	// FIXME
	return 16384;
}

void EnjoyDigitalGenericOscilloscope::SetSampleDepth(uint64_t /*depth*/)
{
	// FIXME
}

void EnjoyDigitalGenericOscilloscope::SetSampleRate(uint64_t /*rate*/)
{
	// FIXME
}

void EnjoyDigitalGenericOscilloscope::SetTriggerOffset(int64_t /*offset*/)
{
	// FIXME
}

int64_t EnjoyDigitalGenericOscilloscope::GetTriggerOffset()
{
	// FIXME
	return 0;
}

bool EnjoyDigitalGenericOscilloscope::IsInterleaving()
{
	return false;
}

bool EnjoyDigitalGenericOscilloscope::SetInterleaving(bool /*combine*/)
{
	return false;
}

void EnjoyDigitalGenericOscilloscope::PullTrigger()
{
	// Clear out any triggers of the wrong type
	if( (m_trigger != NULL) && (dynamic_cast<EdgeTrigger*>(m_trigger) != NULL) )
	{
		delete m_trigger;
		m_trigger = NULL;
	}

	// Create a new trigger if necessary
	if(m_trigger == NULL)
		m_trigger = new EdgeTrigger(this);
	EdgeTrigger* et = dynamic_cast<EdgeTrigger*>(m_trigger);

	// Default setup
	et->SetInput(0, StreamDescriptor(m_channels[0], 0), true);
	et->SetLevel(0.5);
	et->SetType(EdgeTrigger::EDGE_RISING);
}

void EnjoyDigitalGenericOscilloscope::PushTrigger()
{
	// no-op for now
}
