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

namespace gr {
  namespace digital {

    correlate_and_sync_cc::sptr
    correlate_and_sync_cc::make(const std::vector<gr_complex> &symbols,
                                const std::vector<gr_complex> &filter,
                                unsigned int sps)
    {
      return gnuradio::get_initial_sptr
        (new correlate_and_sync_cc_impl(symbols, filter, sps));
    }

    correlate_and_sync_cc_impl::correlate_and_sync_cc_impl(const std::vector<gr_complex> &symbols,
                                                           const std::vector<gr_complex> &filter,
                                                           unsigned int sps)
      : sync_block("correlate_and_sync_cc",
                   io_signature::make(1, 1, sizeof(gr_complex)),
                   io_signature::make(2, 2, sizeof(gr_complex)))
    {
      d_last_index = 0;

      d_sps = sps;
      int nt = filter.size() / sps;
      std::vector<kernel::fir_filter_ccc*> mf;
      std::vector<std::vector<gr_complex> > taps(sps);
      std::vector<gr_complex> vtaps(nt+1, 0);
      std::vector<gr_complex> padding((nt+1)/2, 0);
      std::vector<gr_complex> padded_symbols = symbols;
      padded_symbols.insert(padded_symbols.begin(), padding.begin(), padding.end());

      for(unsigned n = 0; n < sps; n++) {
        mf.push_back(new kernel::fir_filter_ccc(1,vtaps));
	taps[n].resize(nt+1);
      }
      for(size_t i = 0; i < filter.size(); i++) {
	taps[i % sps][i / sps] = filter[i];
      }
      for(unsigned n = 0; n < sps; n++) {
	mf[n]->set_taps(taps[n]);
      }

      std::string printable = "";
      size_t j = 0;
      d_symbols.resize(sps*padded_symbols.size());
      for(size_t i = 0; i < symbols.size(); i++) {
	for(size_t nf = 0; nf < sps; nf++) {
	  d_symbols[j + nf] = mf[nf]->filter(&padded_symbols[i]);
          printable += str(boost::format("%1% + %2%j, ") 
                           % d_symbols[j + nf].real() % d_symbols[j + nf].imag());
	}
	j += sps;
      }

      //GR_LOG_DEBUG(d_logger, printable);

      std::reverse(d_symbols.begin(), d_symbols.end());
      d_thresh = 0.9*powf((float)d_symbols.size(), 2.0f);

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

      memcpy(out, in, sizeof(gr_complex)*noutput_items);
      //d_filter->filter(noutput_items, corr, in);
      d_filter->filterN(corr, in, noutput_items);
      
      std::vector<float> corr_mag(noutput_items);
      volk_32fc_magnitude_squared_32f(&corr_mag[0], corr, noutput_items);

      //for(int i = 0; i < noutput_items; i++) {
      int i = 0;
      while(i < noutput_items) {
        if(corr_mag[i] > d_thresh) {
          float nom = 0, den = 0;
          for(int s = 0; s < 3; s++) {
            //GR_LOG_DEBUG(d_logger, boost::format("%1%:  %2%") % (i+s-1) % corr_mag[i+s]);
            nom += (s+1)*corr_mag[i+s];
            den += corr_mag[i+s];
          }

          float center = nom / den;
          int index = i+1;

          float distance = (d_symbols.size() - 1.0) / 2.0;
          distance /= d_sps;
          distance = distance - int(distance);

          center = (center - 2.0) + distance;
          if(center > d_sps/2)
            center -= d_sps;
          //if(center < 0)
          //  center += 1;
          GR_LOG_DEBUG(d_logger, boost::format("index: %1%:  %2%") 
                       % (nitems_written(0) + index) % center);

          //int diff = nitems_written(0) + index - d_last_index;
          //GR_LOG_DEBUG(d_logger, boost::format("diff: %1%") % (diff));
          //d_last_index = index + nitems_written(0);
          
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

    //[1,-1,1,-1,1,1,-1,-1,1,1,-1,1,1,1,-1,1,1,-1,1,-1,-1,1,-1,-1,1,1,1,-1,-1,-1,1,-1,1,1,1,1,-1,-1,1,-1,1,-1,-1,-1,1,1,-1,-1,-1,-1,1,-1,-1,-1,-1,-1,1,1,1,1,1,1,-1,-1]

  } /* namespace digital */
} /* namespace gr */

