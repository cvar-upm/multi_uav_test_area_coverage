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

#if !defined(SEQUENCE_PAIR_SEQUENCE_HEADER_IS_INCLUDED)
#define SEQUENCE_PAIR_SEQUENCE_HEADER_IS_INCLUDED

#include "sequence/while_sequence.hpp"

namespace seq {

template <typename Sequence0, typename Sequence1>
class pair_sequence {
public:
    using sequence_0_type = Sequence0;
    using sequence_1_type = Sequence1;
    using value_type = std::common_type_t<typename sequence_0_type::value_type, typename sequence_1_type::value_type>;

    template <typename Sequence0Forward, typename Sequence1Forward>
    pair_sequence(Sequence0Forward &&sequence_0, Sequence1Forward &&sequence_1) : m_sequence_0(std::forward<Sequence0Forward>(sequence_0)), m_sequence_1(std::forward<Sequence1Forward>(sequence_1)) { }

    std::optional<value_type> operator*() const;

    pair_sequence &operator++();

    pair_sequence operator++(int);

    std::size_t size() const { return m_sequence_0.size() + m_sequence_1.size(); };

private:
    sequence_0_type m_sequence_0;
    sequence_1_type m_sequence_1;
};



template <typename Sequence0, typename Sequence1>
pair_sequence<std::decay_t<Sequence0>, std::decay_t<Sequence1>> make_pair_sequence(Sequence0 &&sequence_0, Sequence1 &&sequence_1) {
    return pair_sequence<std::decay_t<Sequence0>, std::decay_t<Sequence1>>(std::forward<Sequence0>(sequence_0), std::forward<Sequence1>(sequence_1));
}



template <typename Sequence0, typename Sequence1>
auto operator+(const pair_sequence<Sequence0, Sequence1> &sequence, const typename pair_sequence<Sequence0, Sequence1>::value_type &value) {
    return seq::make_pair_sequence(sequence, seq::lift(value));
}

template <typename Sequence0, typename Sequence1>
auto operator+(const typename pair_sequence<Sequence0, Sequence1>::value_type &value, const pair_sequence<Sequence0, Sequence1> &sequence) {
    return seq::make_pair_sequence(seq::lift(value), sequence);
}



template <typename Sequence0, typename Sequence1>
std::ostream &operator<<(std::ostream &stream, const pair_sequence<Sequence0, Sequence1> &sequence) {
    auto lambda = [&stream](const auto &value) {
        stream << value;
    };

    seq::for_each(sequence, lambda);

    return stream;
}



template <typename Sequence0, typename Sequence1>
auto pair_sequence<Sequence0, Sequence1>::operator*() const -> std::optional<value_type> {
    if(auto optional = *m_sequence_0) {
        return std::optional<value_type>(std::move(optional));
    }

    return std::optional<value_type>(*m_sequence_1);
}

template <typename Sequence0, typename Sequence1>
pair_sequence<Sequence0, Sequence1> &pair_sequence<Sequence0, Sequence1>::operator++() {
    if(seq::size(m_sequence_0)) {
        ++m_sequence_0;
    } else {
        ++m_sequence_1;
    }

    return *this;
}

template <typename Sequence0, typename Sequence1>
pair_sequence<Sequence0, Sequence1> pair_sequence<Sequence0, Sequence1>::operator++(int) {
    auto auxiliar = *this;

    operator++();

    return auxiliar;
}



template <typename Callable0, typename Callable1, typename... Sequence>
auto operator+(const while_sequence<Callable0, Callable1, Sequence...> &sequence, const typename while_sequence<Callable0, Callable1, Sequence...>::value_type &value) {
    return seq::make_pair_sequence(sequence, seq::lift(value));
}

template <typename Callable0, typename Callable1, typename... Sequence>
auto operator+(const typename while_sequence<Callable0, Callable1, Sequence...>::value_type &value, const while_sequence<Callable0, Callable1, Sequence...> &sequence) {
    return seq::make_pair_sequence(seq::lift(value), sequence);
}

}

#endif
