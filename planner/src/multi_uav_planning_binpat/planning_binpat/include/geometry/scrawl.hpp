/*!******************************************************************************
 * \authors   David Eugenio <david.eugenioq@alumnos.upm.es>
 * \copyright Copyright (c) 2022 Universidad Politecnica de Madrid
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#if !defined(GEOMETRY_SCRAWL_HEADER_IS_INCLUDED)
#define GEOMETRY_SCRAWL_HEADER_IS_INCLUDED

#include "geometry/chain.hpp"

namespace gmt {

template <typename Sequence>
class scrawl {
public:
    using sequence_type = Sequence;
    using chain_type = typename sequence_type::value_type;
    using point_type = typename chain_type::point_type;
    using value_type = chain_type;

    template <typename SequenceForward>
    scrawl(utl::forward_t, SequenceForward &&sequence) : m_sequence(std::forward<SequenceForward>(sequence)) { }

    const sequence_type &sequence() const { return m_sequence; }

    sequence_type &sequence() { return m_sequence; }

    std::optional<value_type> operator*() const { return *m_sequence; }

    scrawl &operator++();

    scrawl operator++(int);

    std::size_t size() const { return m_sequence.size(); };

private:
    sequence_type m_sequence;
};



template <typename Sequence>
std::ostream &operator<<(std::ostream &stream, const scrawl<Sequence> &scrawl) {
    return stream << scrawl.sequence();
}



template <typename Sequence>
scrawl<Sequence> &scrawl<Sequence>::operator++() {
    ++m_sequence;

    return *this;
}

template <typename Sequence>
scrawl<Sequence> scrawl<Sequence>::operator++(int) {
    auto auxiliar = *this;

    operator++();

    return auxiliar;
}



template <typename Sequence>
scrawl<std::decay_t<Sequence>> make_scrawl(Sequence &&sequence) {
    return scrawl<std::decay_t<Sequence>>(utl::forward, std::forward<Sequence>(sequence));
}



template <typename Sequence>
auto pop(const scrawl<Sequence> &scrawl) {
    auto lambda = [](const auto &chain) {
        return pop(chain);
    };

    return gmt::make_scrawl(seq::transform(lambda, scrawl));
}



namespace scrwl {

template <typename Scrawl>
auto midpoints(Scrawl &&scrawl) {
    auto lambda = [](const auto &chain_0, auto &&chain_1) {
        return gmt::chn::midpoint(chain_0, chain_1).value();
    };

    auto next = seq::next(scrawl);

    return seq::transform(lambda, std::forward<Scrawl>(scrawl), std::move(next));
}



template <typename Chain>
auto slices(seq::as_sequence_t, Chain &&chain, distance separation, distance length) {
    auto lambda = [chain](auto &&sequence) {
        return gmt::make_chain(seq::make_subsequebce_sequence(chain, std::forward<decltype(sequence)>(sequence)));
    };

    return gmt::make_scrawl(seq::transform(std::move(lambda), gmt::chn::slices(std::forward<Chain>(chain), separation, length)));
}

template <typename Chain>
auto splits(Chain &&chain, std::size_t count, distance separation) {
    if(!count) {
        throw std::logic_error(__func__);
    }

    double length = gmt::chn::length(chain);
    auto separation_length = separation * (count - 1);

    if(separation_length >= length) {
        throw std::logic_error(__func__);
    }

    auto availiable_length = length - separation_length;
    auto sublength = static_cast<gmt::distance>(availiable_length / count);

    sublength *= .999f;

    return gmt::scrwl::slices(seq::as_sequence, std::forward<Chain>(chain), separation, sublength);
}

}

}

#endif
