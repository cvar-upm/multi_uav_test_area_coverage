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

#if !defined(SEQUENCE_CACHED_SEQUENCE_HEADER_IS_INCLUDED)
#define SEQUENCE_CACHED_SEQUENCE_HEADER_IS_INCLUDED

#include "sequence/pair_sequence.hpp"

namespace seq {

template <typename Container, typename Sequence>
class cached_sequence {
public:
    using container_type = Container;
    using sequence_type = Sequence;
    using value_type = typename sequence_type::value_type;

    template <typename ContainerForward, typename SequenceForward>
    cached_sequence(ContainerForward &&container, SequenceForward &&sequence) : m_container(std::forward<ContainerForward>(container)), m_sequence(std::forward<SequenceForward>(sequence)) { }

    std::optional<value_type> operator*() const { return *m_sequence; }

    cached_sequence &operator++();

    cached_sequence operator++(int);

    std::size_t size() const { return m_sequence.size(); }

private:
    container_type m_container;
    sequence_type m_sequence;
};



template <typename Container, typename Sequence>
auto operator+(const cached_sequence<Container, Sequence> &sequence, const typename cached_sequence<Container, Sequence>::value_type &value) {
    return seq::make_pair_sequence(sequence, seq::lift(value));
}

template <typename Container, typename Sequence>
auto operator+(const typename cached_sequence<Container, Sequence>::value_type &value, const cached_sequence<Container, Sequence> &sequence) {
    return seq::make_pair_sequence(seq::lift(value), sequence);
}



template <typename Container, typename Sequence>
std::ostream &operator<<(std::ostream &stream, const cached_sequence<Container, Sequence> &sequence) {
    auto lambda = [&stream](const auto &value) {
        stream << value;
    };

    seq::for_each(sequence, lambda);

    return stream;
}



template <typename Container, typename Sequence>
cached_sequence<Container, Sequence> &cached_sequence<Container, Sequence>::operator++() {
    ++m_sequence;

    return *this;
}

template <typename Container, typename Sequence>
cached_sequence<Container, Sequence> cached_sequence<Container, Sequence>::operator++(int) {
    auto auxiliar = *this;

    operator++();

    return auxiliar;
}



template <typename Container, typename Sequence>
auto make_cached_sequence(Container &&container, Sequence &&sequence) {
    return cached_sequence<std::decay_t<Container>, std::decay_t<Sequence>>(std::forward<Container>(container), std::forward<Sequence>(sequence));
}

}

#endif
