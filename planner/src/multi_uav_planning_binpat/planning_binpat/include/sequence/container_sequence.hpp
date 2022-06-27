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

#if !defined(SEQUENCE_CONTAINER_SEQUENCE_HEADER_IS_INCLUDED)
#define SEQUENCE_CONTAINER_SEQUENCE_HEADER_IS_INCLUDED

#include <vector>

#include "sequence/range_sequence.hpp"

namespace seq {

template <typename Container>
class container_sequence {
public:
    using container_type = Container;
    using value_type = typename range_sequence<typename container_type::const_iterator>::value_type;

    template <typename ContainerForward>
    container_sequence(utl::forward_t, ContainerForward &&container) : m_container(std::forward<ContainerForward>(container)) { }

    auto &container() & { return m_container; }

    auto &&container() && { return std::move(m_container); }

    const auto &container() const & { return m_container; }

    auto lightweight() const { return seq::range_sequence(m_container.begin() + m_index, m_container.end()); }

    std::optional<value_type> operator*() const { return m_index < m_container.size() ? std::make_optional(m_container[m_index]) : std::optional<value_type>(); }

    container_sequence &operator++();

    container_sequence operator++(int);

    std::size_t size() const { return m_container.size() - m_index; }

private:
    container_type m_container;
    typename container_type::size_type m_index{ };
};



template <typename Container>
auto operator+(const container_sequence<Container> &sequence, const typename container_sequence<Container>::value_type &value) {
    return seq::make_pair_sequence(sequence, seq::lift(value));
}

template <typename Container>
auto operator+(const typename container_sequence<Container>::value_type &value, const container_sequence<Container> &sequence) {
    return seq::make_pair_sequence(seq::lift(value), sequence);
}



template <typename Container>
std::ostream &operator<<(std::ostream &stream, container_sequence<Container> sequence) {
    auto lambda = [&stream](const auto &value) {
        stream << value;
    };

    seq::for_each(sequence, lambda);

    return stream;
}



template <typename Container>
container_sequence<Container> &container_sequence<Container>::operator++() {
    ++m_index;

    if(m_index == 0u) {
        --m_index;
    }

    return *this;
}

template <typename Container>
container_sequence<Container> container_sequence<Container>::operator++(int) {
    auto auxiliar = *this;

    operator++();

    return auxiliar;
}



template <typename Container>
container_sequence<std::decay_t<Container>> make_container_sequence(Container &&container) {
    return container_sequence<std::decay_t<Container>>(utl::forward, std::forward<Container>(container));
}

template <typename Sequence>
auto make_container_sequence_(Sequence &&sequence) {
    using value_type = typename std::decay_t<Sequence>::value_type;

    auto container = seq::accumulate<std::vector<value_type>>(std::forward<Sequence>(sequence));

    return seq::make_container_sequence(std::move(container));
}

}

#endif
