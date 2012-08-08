/* -*- c++ -*- */
/*
 * Copyright 2012 Free Software Foundation, Inc.
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

#include "generic_filterbank_vcvcf_impl.h"
#include <gr_io_signature.h>
#include <stdio.h>

namespace gr {
  namespace filter {
    
    generic_filterbank_vcvcf::sptr
    generic_filterbank_vcvcf::make(
      const std::vector< std::vector<float> > &taps)
    {
      return gnuradio::get_initial_sptr(new generic_filterbank_vcvcf_impl(taps));
    }

    generic_filterbank_vcvcf_impl::generic_filterbank_vcvcf_impl(
      const std::vector< std::vector<float> > &taps)
      : gr_sync_block("generic_filterbank_vcvcf",
                      gr_make_io_signature(1, 1, sizeof(gr_complex) * taps.size()),
                      gr_make_io_signature(1, 1, sizeof(gr_complex) * taps.size())),
        d_taps(taps)
    {
      // Get number of filters.
      d_nfilts = d_taps.size();
      if (d_nfilts == 0)
        throw std::invalid_argument("The taps vector may not be empty.");
      d_active.resize(d_nfilts);
      
      // Create an FIR filter for each channel.  Initially the taps are set
      // to a single 0.  The 'set_taps' method will set them correctly.
      d_filters = std::vector<kernel::fir_filter_ccf*>(d_nfilts);
      std::vector<float> vtaps(0, 1);
      for(unsigned int i = 0; i < d_nfilts; i++) {
        d_filters[i] = new kernel::fir_filter_ccf(1, vtaps);
      }
      
      // Now, actually set the filters' taps
      set_taps(d_taps);
      set_history(d_ntaps+1);
    }

    generic_filterbank_vcvcf_impl::~generic_filterbank_vcvcf_impl()
    {
      for(unsigned int i = 0; i < d_nfilts; i++) {
        delete d_filters[i];
      }
    }

    void
    generic_filterbank_vcvcf_impl::set_taps(
      const std::vector< std::vector<float> > &taps)
    {
      gruel::scoped_lock guard(d_mutex);
      d_taps = taps;
      // Check that the number of filters is correct.
      if (d_nfilts != d_taps.size())
        throw std::runtime_error("The number of filters is incorrect.");
      // Check that taps contains vectors of taps, where each vector
      // is the same length.
      d_ntaps = d_taps[0].size();
      for (unsigned int i = 1; i < d_nfilts; i++) {
        if (d_taps[i].size() != d_ntaps) {
          throw std::runtime_error("All sets of taps must be of the same length.");
        }
      }
      for(unsigned int i = 0; i < d_nfilts; i++) {
        // If filter taps are all zeros don't bother to crunch the numbers.
        d_active[i] = false;
        for (unsigned int j = 0; j < d_ntaps; j++) {
          if (d_taps[i][j] != 0) {
            d_active[i] = true;
            break;
          }
        }
        d_filters[i]->set_taps(d_taps[i]);
      }

      set_history(d_ntaps+1);
      d_updated = true;
    }

    void
    generic_filterbank_vcvcf_impl::print_taps()
    {
      unsigned int i, j;
      for(i = 0; i < d_nfilts; i++) {
        printf("filter[%d]: [", i);
        for(j = 0; j < d_ntaps; j++) {
          printf(" %.4e", d_taps[i][j]);
        }
        printf("]\n\n");
      }
    }

    std::vector< std::vector<float> >
    generic_filterbank_vcvcf_impl::taps() const
    {
      return d_taps;
    }

    int
    generic_filterbank_vcvcf_impl::work(
      int noutput_items,
      gr_vector_const_void_star &input_items,
      gr_vector_void_star &output_items)
    {
      gruel::scoped_lock guard(d_mutex);

      gr_complex *in = (gr_complex *) input_items[0];
      gr_complex *out = (gr_complex *) output_items[0];

      if(d_updated) {
        d_updated = false;
        return 0;            // history requirements may have changed.
      }
 
      
      gr_complex *working;
      // rmeoved d_ntaps-1
      working = new gr_complex [noutput_items + d_ntaps];

      for (unsigned int i = 0; i < d_nfilts; i++) {
        // Only call the filter method on active filters.
        if (d_active[i]) {
          for (unsigned int j = 0; j < noutput_items + d_ntaps-1; j++) {
            unsigned int p = i + j*d_nfilts;
            working[j] = in[p];
          }
          for (unsigned int j = 0; j < (unsigned int)(noutput_items); j++) {
            unsigned int p = i + j*d_nfilts;
            out[p] = d_filters[i]->filter(working + j);
          }
        } else {
          // Otherwise just output 0s.
          for (unsigned int j = 0; j < (unsigned int)(noutput_items); j++) {
            unsigned int p = i + j*d_nfilts;
            out[p] = 0;
          }
        }
      }
     
      delete [] working;
      return noutput_items;
    }

  } /* namespace filter */
} /* namespace gr */
