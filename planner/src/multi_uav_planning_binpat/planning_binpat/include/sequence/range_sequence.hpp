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

#if !defined(SEQUENCE_RANGE_SEQUENCE_HEADER_IS_INCLUDED)
#define SEQUENCE_RANGE_SEQUENCE_HEADER_IS_INCLUDED

#include <iterator>

#include "sequence/pair_sequence.hpp"

namespace seq {

template <typename Iterator>
class range_sequence {
public:
    using iterator_type = Iterator;
    using value_type = std::decay_t<typename std::iterator_traits<iterator_type>::value_type>;

    range_sequence(iterator_type begin, iterator_type end) : m_begin(begin), m_end(end) { }

    std::optional<value_type> operator*() const;

    range_sequence &operator++();

    range_sequence operator++(int);

    std::size_t size() const { return m_end - m_begin; }

private:
    iterator_type m_begin;
    iterator_type m_end;
};



template <typename Iterator>
auto operator+(const range_sequence<Iterator> &sequence, const typename range_sequence<Iterator>::value_type &value) {
    return seq::make_pair_sequence(sequence, seq::lift(value));
}

template <typename Iterator>
auto operator+(const typename range_sequence<Iterator>::value_type &value, const range_sequence<Iterator> &sequence) {
    return seq::make_pair_sequence(seq::lift(value), sequence);
}



template <typename Iterator>
std::ostream &operator<<(std::ostream &stream, range_sequence<Iterator> sequence) {
    auto lambda = [&stream](const auto &value) {
        stream << value;
    };

    seq::for_each(sequence, lambda);

    return stream;
}



template <typename Iterator>
auto range_sequence<Iterator>::operator*() const -> std::optional<value_type> {
    return size() ? std::make_optional<value_type>(*m_begin) : std::optional<value_type>();
}

template <typename Iterator>
range_sequence<Iterator> &range_sequence<Iterator>::operator++() {
    if(size()) {
        ++m_begin;
    }

    return *this;
}

template <typename Iterator>
range_sequence<Iterator> range_sequence<Iterator>::operator++(int) {
    auto auxiliar = *this;

    operator++();

    return auxiliar;
}



template <typename Container>
auto make_range_sequence(const Container &container) {
    return range_sequence(container.cbegin(), container.cend());
}

}

#endif
