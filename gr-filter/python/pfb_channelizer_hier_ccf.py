"""
Implements a hierarchical polyphase filterbank channelizer.

The advantage of this over the standard pfb_channelizer_ccf block is
the number-crunching is distributed over several signal-processing blocks
which is helpful if the standard pfb_channelizer_ccf is maxing out one
of your cores.
"""

import math

from gnuradio import gr
import filter_swig as filter
import optfir

# FIXME: Would be nice to integrate python logging module into gnuradio.
class logger(object):

    @classmethod
    def debug(cls, x):
        pass
        #print(x)

class pfb_channelizer_hier_ccf(gr.hier_block2):
    """
    Make a Polyphase Filter channelizer (complex in, complex out, floating-point taps)

    Args:
        n_chans - The number of channels to split into.
        n_filterbanks - The number of filterbank blocks to use (default=2).
        taps: The taps to use.  If this is `None` then taps are generated using optfir.low_pass.
        outchans - Which channels to output streams for (a list of integers) (default is all channels).
        atten: Stop band attenuation.
        bw: The fraction of the channel you want to keep.
        tb: Transition band with as fraction of channel width.
        ripple: Pass band ripple in dB.
    """

    def __init__(self, n_chans, n_filterbanks=1, taps=None, outchans=None, atten=100, bw=1.0, tb=0.2, ripple=0.1):
        if outchans is None:
            outchans = range(n_chans)
	gr.hier_block2.__init__(self, "pfb_channelizer_hier_ccf",
				gr.io_signature(1, 1, gr.sizeof_gr_complex),
				gr.io_signature(len(outchans), len(outchans), gr.sizeof_gr_complex))
        if taps is None:
            taps = optfir.low_pass(1, n_chans, bw, bw+tb, ripple, atten)
        taps = list(taps)
        extra_taps = int(math.ceil(1.0*len(taps)/n_chans)*n_chans - len(taps))
        taps = taps + [0] * extra_taps
        # Make taps for each channel
        chantaps = [list(reversed(taps[i: len(taps): n_chans])) for i in range(0, n_chans)]
        # Convert the input stream into a stream of vectors.
        self.s2v = gr.stream_to_vector(gr.sizeof_gr_complex, n_chans)
        # Create a mapping to separate out each filterbank (a group of channels to be processed together)
        # And a list of sets of taps for each filterbank.
        low_cpp = int(n_chans/n_filterbanks)
        extra = n_chans - low_cpp*n_filterbanks
        cpps = [low_cpp+1]*extra + [low_cpp]*(n_filterbanks-extra)
        splitter_mapping = []
        filterbanktaps = []
        total = 0
        for cpp in cpps:
            splitter_mapping.append([(0, i) for i in range(total, total+cpp)])
            filterbanktaps.append(chantaps[total: total+cpp])
            total += cpp
        logger.debug("Creating filterbanks with numbers of channels {0}".format(cpps))
        assert(total == n_chans)
        # Split the stream of vectors in n_filterbanks streams of vectors.
        self.splitter = gr.vector_map(gr.sizeof_gr_complex, [n_chans], splitter_mapping)
        # Create the filterbanks
        self.fbs = [filter.generic_filterbank_vcvcf(taps) for taps in filterbanktaps]
        # Combine the streams of vectors back into a single stream of vectors.
        combiner_mapping = [[]]
        for i, cpp in enumerate(cpps):
            for j in range(cpp):
                combiner_mapping[0].append((i, j))
        self.combiner = gr.vector_map(gr.sizeof_gr_complex, cpps, combiner_mapping)
        self.prefft_snk = gr.vector_sink_c(n_chans)
        self.connect(self.combiner, self.prefft_snk)
        # Add the final FFT to the channelizer.
        self.fft = gr.fft_vcc(n_chans, forward=True, window=[1.0]*n_chans)
        # Select the desired channels
        if outchans != range(n_chans):
            selector_mapping = [[(0, i) for i in outchans]]
            self.selector = gr.vector_map(gr.sizeof_gr_complex, [n_chans], selector_mapping)
        # Convert stream of vectors to a normal stream.
        self.v2ss = gr.vector_to_streams(gr.sizeof_gr_complex, len(outchans))
        self.connect(self, self.s2v, self.splitter)
        for i in range(0, n_filterbanks):
            self.connect((self.splitter, i), self.fbs[i], (self.combiner, i))
        self.connect(self.combiner, self.fft)
        if outchans != range(n_chans):
            self.connect(self.fft, self.selector, self.v2ss)
        else:
            self.connect(self.fft, self.v2ss)
        for i in range(0, len(outchans)):
            self.connect((self.v2ss, i), (self, i))
            


