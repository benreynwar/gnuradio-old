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


#ifndef INCLUDED_FILTER_GENERIC_FILTERBANK_VCVCF_H
#define INCLUDED_FILTER_GENERIC_FILTERBANK_VCVCF_H

#include <filter/api.h>
#include <gr_sync_block.h>

namespace gr {
  namespace filter {

    /*!
     * \class generic_filterbank_vcvcf
     *
     * \brief A filter bank with generic taps.
     *
     * \ingroup filter_blk
     *
     * This block takes in a vector of N complex inputs, passes
     * them through N FIR filters, and outputs a vector of N complex
     * outputs.
     *
     * The only advantage of using this block over N individual
     * FIR filter blocks is that it places less of a load on the 
     * scheduler.
     *
     * The number of filters cannot be changed dynamically, however
     * filters can be deactivated (i.e. no processing is done for
     * them) by passing a vector of filter taps containing all zeros
     * to them.  In this case their entry in the output vector is a
     * zero.
     */
    class FILTER_API generic_filterbank_vcvcf : virtual public gr_sync_block
    {
    public:
      typedef boost::shared_ptr<generic_filterbank_vcvcf> sptr;
      /*!
       * Build the generic filterbank.
       * \param taps (vector/list of vectors/lists of floats) A vector of taps
                     is given for each filter.  These vectors must all have the
                     same length.  No calculation is performed for
                     vectors of taps containing all zeros.
      */
      static sptr make(const std::vector< std::vector<float> > &taps);

      /*!
       * Resets the filterbank's filter taps.
       * \param taps (vector/list vectors/lists of floats) The new taps.
       */
      virtual void set_taps(const std::vector< std::vector<float> > &taps) = 0;

      /*!
       * Print all of the filterbank taps to screen.
       */
      virtual void print_taps() = 0;
      
      /*!
       * Return a vector<vector<>> of the filterbank taps
       */
      virtual std::vector<std::vector<float> > taps() const = 0;
    };
    
  } /* namespace filter */
} /* namespace gr */

#endif /* INCLUDED_FILTER_GENERIC_FILTERBANK_VCVCF_H */
