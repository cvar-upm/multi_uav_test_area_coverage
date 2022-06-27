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

#if !defined(LABEL_ALGORITHM_HEADER_IS_INCLUDED)
#define LABEL_ALGORITHM_HEADER_IS_INCLUDED

#include "label/point.hpp"

namespace lbl {

namespace chn {

template <typename Chain>
auto make_chain(Chain &&chain) {
    auto lambda = [](const auto &point) {
        return lbl::make_point(true, point);
    };

    return gmt::make_chain(seq::transform(lambda, std::forward<Chain>(chain)));
}



template <seq::endpoint Endpoint, typename Chain, std::enable_if_t<Endpoint == seq::endpoint::front, bool> = true>
std::size_t trim(Chain &&chain) {
    auto lambda = [](const auto &point) {
        return point.getlabel();
    };

    return seq::find_if(std::forward<Chain>(chain), lambda);
}

template <seq::endpoint Endpoint, typename Chain, std::enable_if_t<Endpoint == seq::endpoint::back, bool> = true>
std::size_t trim(Chain &&chain) {
    return lbl::chn::trim<seq::endpoint::front>(gmt::make_chain(seq::reverse(std::forward<Chain>(chain))));
}



template <typename Chain>
seq::subsequence trim(Chain &&chain) {
    auto front = lbl::chn::trim<seq::endpoint::front>(chain);

    auto back = lbl::chn::trim<seq::endpoint::back>(chain);

    return seq::subsequence(front, seq::size(std::forward<Chain>(chain)) - back);
}

template <typename Chain>
auto trim(seq::as_sequence_t, Chain &&chain) {
    auto subsequence = lbl::chn::trim(chain);

    return gmt::make_chain(seq::make_subsequebce_sequence(std::forward<Chain>(chain), subsequence));
}

}



namespace scrwl {

template <typename Scrawl>
auto make_scrawl(Scrawl &&scrawl) {
    auto lambda = [](auto &&chain) {
        return lbl::chn::make_chain(std::forward<decltype(chain)>(chain));
    };

    return gmt::make_scrawl(seq::transform(lambda, std::forward<Scrawl>(scrawl)));
}



template <typename Scrawl>
auto trim(Scrawl &&scrawl) {
    auto lambda = [](auto &&chain) {
        return lbl::chn::trim(seq::as_sequence, std::forward<decltype(chain)>(chain));
    };

    return gmt::make_scrawl(seq::transform(lambda, std::forward<Scrawl>(scrawl)));
}



template <typename Scrawl>
auto link(Scrawl &&scrawl) {
    using unit_type = decltype(seq::lift(std::declval<typename std::decay_t<Scrawl>::point_type>()));
    using sequence_type = seq::variant_sequence<typename std::decay_t<Scrawl>::chain_type, unit_type>;

    auto midpoints = gmt::scrwl::midpoints(scrawl);

    auto lambda_0 = [](const auto &point) {
        return sequence_type(utl::forward, seq::lift(lbl::falset(point)));
    };

    auto lambda_1 = [](auto &&chain) {
        return sequence_type(utl::forward, std::forward<decltype(chain)>(chain));
    };

    auto intersperse = seq::intersperse(seq::transform(lambda_1, std::forward<Scrawl>(scrawl)), seq::transform(lambda_0, std::move(midpoints)));

    return gmt::make_chain(seq::flatten(std::move(intersperse)));
}

}

}

#endif
