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

#if !defined(SEQUENCE_TRANSFORM_SEQUENCE_HEADER_IS_INCLUDED)
#define SEQUENCE_TRANSFORM_SEQUENCE_HEADER_IS_INCLUDED

#include <algorithm>

#include "sequence/pair_sequence.hpp"

namespace seq {

template <typename Callable, typename... Sequence>
class transform_sequence {
public:
    using callable_type = Callable;
    using value_type = utl::call_t<callable_type, typename Sequence::value_type...>;

    template <typename CallableForward, typename... SequenceForward>
    transform_sequence(utl::forward_t, CallableForward &&callable, SequenceForward &&... sequence) : m_callable(std::forward<CallableForward>(callable)), m_sequences(std::forward<SequenceForward>(sequence)...) { }

    std::optional<value_type> operator*() const;

    transform_sequence &operator++();

    transform_sequence operator++(int);

    std::size_t size() const {
        auto lambda = [](const auto &... sequence) {
            return std::min({sequence.size()...});
        };

        return std::apply(lambda, m_sequences);
    }

private:
    callable_type m_callable;
    std::tuple<Sequence...> m_sequences;
};



template <typename Callable, typename... Sequence>
auto operator+(const transform_sequence<Callable, Sequence...> &sequence, const typename transform_sequence<Callable, Sequence...>::value_type &value) {
    return seq::make_pair_sequence(sequence, seq::lift(value));
}

template <typename Callable, typename... Sequence>
auto operator+(const typename transform_sequence<Callable, Sequence...>::value_type &value, const transform_sequence<Callable, Sequence...> &sequence) {
    return seq::make_pair_sequence(seq::lift(value), sequence);
}



template <typename Callable, typename... Sequence>
std::ostream &operator<<(std::ostream &stream, transform_sequence<Callable, Sequence...> sequence) {
    auto lambda = [&stream](const auto &value) {
        stream << value;
    };

    seq::for_each(sequence, lambda);

    return stream;
}



template <typename Callable, typename... Sequence>
auto transform_sequence<Callable, Sequence...>::operator*() const -> std::optional<value_type> {
    auto lambda = [this](const auto &... sequence) {
        return utl::opt::transform(m_callable, *sequence...);
    };

    return std::apply(lambda, m_sequences);
}

template <typename Callable, typename... Sequence>
transform_sequence<Callable, Sequence...> &transform_sequence<Callable, Sequence...>::operator++() {
    auto lambda = [](auto &... sequence) {
        (++sequence, ...);
    };

    std::apply(lambda, m_sequences);

    return *this;
}

template <typename Callable, typename... Sequence>
transform_sequence<Callable, Sequence...> transform_sequence<Callable, Sequence...>::operator++(int) {
    auto auxiliar = *this;

    operator++();

    return auxiliar;
}



template <typename Callable, typename... Sequence>
transform_sequence<std::decay_t<Callable>, std::decay_t<Sequence>...> transform(Callable &&callable, Sequence &&... sequence) {
    return seq::transform_sequence<std::decay_t<Callable>, std::decay_t<Sequence>...>(utl::forward, std::forward<Callable>(callable), std::forward<Sequence>(sequence)...);
}


}

#endif
