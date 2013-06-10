/* -*- c++ -*- */
/* 
 * Copyright 2013 Free Software Foundation, Inc.
 * 
 * This file is part of GNU Radio
 * 
 * GNU Radio is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * GNU Radio is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include <gnuradio/math.h>
#include "correlate_and_sync_cc_impl.h"
#include <volk/volk.h>
#include <boost/format.hpp>
#include <boost/math/special_functions/round.hpp>
#include <gnuradio/filter/pfb_arb_resampler.h>
#include <gnuradio/filter/firdes.h>

namespace gr {
  namespace digital {

    correlate_and_sync_cc::sptr
    correlate_and_sync_cc::make(const std::vector<gr_complex> &symbols,
                                const std::vector<float> &filter,
                                unsigned int sps, unsigned int nfilts)
    {
      return gnuradio::get_initial_sptr
        (new correlate_and_sync_cc_impl(symbols, filter, sps, nfilts));
    }

    correlate_and_sync_cc_impl::correlate_and_sync_cc_impl(const std::vector<gr_complex> &symbols,
                                                           const std::vector<float> &filter,
                                                           unsigned int sps, unsigned int nfilts)
      : sync_block("correlate_and_sync_cc",
                   io_signature::make(1, 1, sizeof(gr_complex)),
                   io_signature::make(2, 2, sizeof(gr_complex)))
    {
      d_last_index = 0;
      d_sps = sps;

      // The number of complete filters that can be made.
      unsigned int complete_filters = filter.size()/nfilts;
      // The number of extra 0's required to pad it to a multiple of nfilts.
      unsigned int extras = filter.size() - complete_filters*nfilts;
      std::vector<float> padded_filter = filter;
      if (extras % 2 == 0) {
        std::vector<float> padding(extras/2, 0);
        padded_filter.insert(padded_filter.begin(), padding.begin(), padding.end());
        padded_filter.insert(padded_filter.end(), padding.begin(), padding.end());
      } else {
        std::vector<float> padding_start(extras/2+1, 0);
        std::vector<float> padding_end(extras/2, 0);
        padded_filter.insert(padded_filter.begin(), padding_start.begin(), padding_start.end());
        padded_filter.insert(padded_filter.end(), padding_end.begin(), padding_end.end());
      }

      // We want to add padding to the beginning of the symbols so we can do the convolution of the symbols
      // with the pulse shape.
      std::vector<gr_complex> padding(padded_filter.size()/nfilts, 0);
      std::vector<gr_complex> padded_symbols = symbols;
      padded_symbols.insert(padded_symbols.begin(), padding.begin(), padding.end());
      padded_symbols.insert(padded_symbols.end(), padding.begin(), padding.end());

      d_symbols.resize(d_sps*padded_symbols.size());
      filter::kernel::pfb_arb_resampler_ccf resamp(d_sps, padded_filter, nfilts);
      resamp.print_taps();
      resamp.filter(&d_symbols[0], &padded_symbols[0], d_sps*padded_symbols.size());

      std::reverse(d_symbols.begin(), d_symbols.end());
      d_thresh = 0.9*powf((float)d_symbols.size(), 2.0f);
      d_center_first_symbol = (padding.size() + 0.5) * d_sps;

      //d_filter = new kernel::fft_filter_ccc(1, d_symbols);
      d_filter = new kernel::fir_filter_ccc(1, d_symbols);

      set_history(d_filter->ntaps());

      const int alignment_multiple =
        volk_get_alignment() / sizeof(gr_complex);
      set_alignment(std::max(1,alignment_multiple));
    }
    
    correlate_and_sync_cc_impl::~correlate_and_sync_cc_impl()
    {
      delete d_filter;
    }

    std::vector<gr_complex>
    correlate_and_sync_cc_impl::symbols() const
    {
      return d_symbols;
    }

    void
    correlate_and_sync_cc_impl::set_symbols(const std::vector<gr_complex> &symbols)
    {
      gr::thread::scoped_lock lock(d_setlock);
      d_symbols = symbols;
      d_filter->set_taps(symbols);
    }

    int
    correlate_and_sync_cc_impl::work(int noutput_items,
                                     gr_vector_const_void_star &input_items,
                                     gr_vector_void_star &output_items)
    {
      gr::thread::scoped_lock lock(d_setlock);

      const gr_complex *in = (gr_complex *)input_items[0];
      gr_complex *out = (gr_complex*)output_items[0];
      gr_complex *corr = (gr_complex*)output_items[1];

      // FIXME: Shouldn't copy last filter worth of data across.
      // We want to process this again on next call to work.
      // This would prevent the preamble falling across two work functions.
      memcpy(out, in, sizeof(gr_complex)*noutput_items);
      //d_filter->filter(noutput_items, corr, in);
      d_filter->filterN(corr, in, noutput_items);
      
      std::vector<float> corr_mag(noutput_items);
      volk_32fc_magnitude_squared_32f(&corr_mag[0], corr, noutput_items);

      int i = 0;
      while(i < noutput_items) {

        if ((corr_mag[i] > d_thresh) && (corr_mag[i+1] > corr_mag[i]) && (corr_mag[i+1] > corr_mag[i+2])) {
          float nom = 0, den = 0;
          for(int s = 0; s < 3; s++) {
            nom += (s+1)*corr_mag[i+s];
            den += corr_mag[i+s];
          }
          float center = nom / den;
          int index = i+1;
          center = - (center - 2.0) - d_center_first_symbol;
          while (center < d_sps/2)
            center += d_sps;
          while (center > d_sps/2)
            center -= d_sps;
          
          GR_LOG_DEBUG(d_logger, boost::format("index: %1%:  %2%") 
                       % (nitems_written(0) + index) % center);

          float phase = fast_atan2f(corr[index].imag(), corr[index].real());
          add_item_tag(0, nitems_written(0) + index, pmt::intern("phase_est"),
                       pmt::from_double(phase), pmt::intern(alias()));
          add_item_tag(0, nitems_written(0) + index, pmt::intern("time_est"),
                       pmt::from_double(center),
                       pmt::intern(alias()));
          add_item_tag(0, nitems_written(0) + index, pmt::intern("corr_est"),
                       pmt::from_double(corr_mag[index]), pmt::intern(alias()));

          i += d_sps;
        }
        else
          i++;
      }

      return noutput_items;
    }

  } /* namespace digital */
} /* namespace gr */

