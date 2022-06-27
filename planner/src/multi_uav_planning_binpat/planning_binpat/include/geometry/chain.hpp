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

#if !defined(GEOMETRY_CHAIN_HEADER_IS_INCLUDED)
#define GEOMETRY_CHAIN_HEADER_IS_INCLUDED

#include "sequence.hpp"

#include "geometry/segment.hpp"

namespace gmt {

template <typename Sequence>
class chain {
public:
    using sequence_type = Sequence;
    using point_type = typename sequence_type::value_type;
    using value_type = point_type;
    using continuum_type = segment<point_type>;

    template <typename SequenceForward>
    chain(utl::forward_t, SequenceForward &&sequence) : m_sequence(std::forward<SequenceForward>(sequence)) { }

    sequence_type &sequence() { return m_sequence; }

    const sequence_type &sequence() const { return m_sequence; }

    std::optional<value_type> operator*() const { return *m_sequence; }

    chain &operator++();

    chain operator++(int);

    std::size_t size() const { return m_sequence.size(); };

private:
    sequence_type m_sequence;
};



template <typename Sequence>
chain<std::decay_t<Sequence>> make_chain(Sequence &&sequence) {
    return chain<std::decay_t<Sequence>>(utl::forward, std::forward<Sequence>(sequence));
}



template <typename Sequence>
auto operator+(const chain<Sequence> &chain_, const typename chain<Sequence>::value_type &value) {
    return gmt::make_chain(chain_.sequence() + value);
}

template <typename Sequence>
auto operator+(const typename chain<Sequence>::value_type &value, const chain<Sequence> &chain) {
    return gmt::make_chain(value + chain.sequence());
}



template <typename Sequence>
std::ostream &operator<<(std::ostream &stream, const chain<Sequence> &chain) {
    return stream << chain.sequence();
}



template <typename Sequence>
chain<Sequence> &chain<Sequence>::operator++() {
    ++m_sequence;

    return *this;
}

template <typename Sequence>
chain<Sequence> chain<Sequence>::operator++(int) {
    auto auxiliar = *this;

    operator++();

    return auxiliar;
}



template <typename Sequence>
auto pop(const chain<Sequence> &chain) {
    auto lambda = [](const auto &point) {
        return pop(point);
    };

    return gmt::make_chain(seq::transform(lambda, chain));
}



namespace chn {

template <typename Chain>
distance length(Chain &&chain) {
    auto lambda = [](const auto &segment) {
        return gmt::length(segment);
    };

    return seq::accumulate(seq::transform(lambda, seq::continuum(std::forward<Chain>(chain))));
}

template <typename Chain>
typename std::decay_t<Chain>::value_type midpoint(Chain &&chain) {
    auto size = static_cast<distance>(seq::size(chain));

    return seq::accumulate(std::forward<Chain>(chain)) / size;
}



template <typename Chain0, typename Chain1>
std::optional<typename Chain1::point_type> midpoint(Chain0 &&chain_0, const Chain1 &chain_1) {
    auto lambda = [](const auto &point_0, const auto &point_1) {
        return gmt::pnt::midpoint(point_0, point_1);
    };

    return utl::opt::transform(lambda, seq::last(std::forward<Chain0>(chain_0)), seq::first(chain_1));
}



template <typename Chain, typename Callable>
auto push(Chain &&chain, Callable &&callable) {
    auto lambda = [callable = std::forward<Callable>(callable)](const auto &point) {
        return push(point, callable(point));
    };

    return gmt::make_chain(seq::transform(std::move(lambda), std::forward<Chain>(chain)));
}



template <typename Chain>
std::variant<seq::continuous_size, distance> cut(Chain &&chain, distance length) {
    auto index_sequence = seq::make_index_sequence(chain);

    auto lambda = [](auto length, const auto &segment, auto index) {
        auto lambda = [index](auto percentage) {
            return seq::continuous_size(index, percentage);
        };

        return utl::var::transform(lambda, gmt::cut(segment, length));
    };

    auto sequence = seq::iterate<seq::continuous_size>(lambda, length, seq::continuum(std::forward<Chain>(chain)), std::move(index_sequence));

    if(auto value = seq::last(std::move(sequence))) {
        return value.value();
    } else {
        return length;
    }
}



template <typename Chain>
std::variant<seq::continuous_subsequence, distance> slice(Chain &&chain, distance offset, distance length) {
    auto lambda = [chain, offset, length](const auto &front) {
        auto lambda = [front](const auto &back) {
            return seq::continuous_subsequence(front, back);
        };

        return utl::var::transform(lambda, cut(chain, offset + length));
    };

    return utl::var::and_then(std::move(lambda), cut(std::forward<Chain>(chain), offset));
}

template <typename Chain>
std::optional<seq::continuous_size> intersect(Chain &&chain, const line<typename std::decay_t<Chain>::point_type> &line) {
    auto lambda = [&line](const auto &segment) {
        return gmt::intersect(segment, line);
    };

    return seq::find(seq::continuum(std::forward<Chain>(chain)), lambda);
}

template <typename Chain>
auto slices(Chain &&chain, distance separation, distance length) {
    auto lambda_0 = [chain, length]() {
        return utl::var::forget(gmt::chn::slice(chain, distance{ }, length));
    };

    auto lambda_1 = [chain = std::forward<Chain>(chain), length, separation](const auto &subsequence) {
        auto lambda = [subsequence](const auto &subsequence_) {
            return seq::advance(subsequence_, subsequence.second);
        };

        return utl::var::forget(utl::var::transform(lambda, gmt::chn::slice(seq::next(chain, subsequence.second), separation, length)));
    };

    return seq::make_while_sequence(std::move(lambda_0), std::move(lambda_1));
}



template <typename Chain>
seq::endpoint closest(const Chain &chain, const typename Chain::point_type &point) {
    auto lambda = [&chain, &point](auto endpoint) {
        return gmt::pnt::separation(seq::at(chain, endpoint), point);
    };

    return seq::at(seq::make_enum_sequence<seq::endpoint>(), seq::min(seq::transform(lambda, seq::make_enum_sequence<seq::endpoint>())));
}

}



namespace bchn {

template <typename Bichain, typename Point>
seq::biendpoint closest(const Bichain &bichain, const Point &point) {
    auto lambda = [&bichain, &point](auto biendpoint) {
        return gmt::pnt::separation(seq::at(bichain, biendpoint), point);
    };

    return seq::at(seq::make_enum_sequence<seq::biendpoint>(), seq::min(seq::transform(lambda, seq::make_enum_sequence<seq::biendpoint>())));
}

}

}

#endif
