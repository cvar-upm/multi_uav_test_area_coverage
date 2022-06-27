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

#if !defined(SEQUENCE_FLATTEN_SEQUENCE_HEADER_IS_INCLUDED)
#define SEQUENCE_FLATTEN_SEQUENCE_HEADER_IS_INCLUDED

#include <functional>

#include "sequence/pair_sequence.hpp"

namespace seq {

template <typename Sequence>
class flatten_sequence {
public:
    using sequence_type = Sequence;
    using value_type = typename Sequence::value_type::value_type;

    template <typename SequenceForward>
    flatten_sequence(utl::forward_t, SequenceForward &&sequence) : m_sequence(std::forward<SequenceForward>(sequence)), m_subsequence(*m_sequence) {
        auto lambda = [this](auto &&sequence) {
            if(!*sequence) {
                ++m_sequence;

                auto lambda = [](auto &&sequence) {
                    return seq::size(std::forward<decltype(sequence)>(sequence)) != 0u;
                };

                auto index = seq::find_if(m_sequence, lambda);

                if(index != seq::size(m_sequence)) {
                    seq::advance(m_sequence, index);

                    m_subsequence.emplace(seq::front(m_sequence));
                }
            }

            return nullptr;
        };

        utl::opt::and_then(lambda, m_subsequence);
    }

    sequence_type sequence() const { return m_sequence; }

    std::optional<value_type> operator*() const;

    std::optional<value_type> operator->() const { return operator*(); }

    flatten_sequence &operator++();

    flatten_sequence operator++(int);

    std::size_t size() const { return seq::raw_size(*this); }

private:
    sequence_type m_sequence;
    std::optional<typename sequence_type::value_type> m_subsequence;
};



template <typename Sequence>
auto operator+(const flatten_sequence<Sequence> &sequence, const typename flatten_sequence<Sequence>::value_type &value) {
    return seq::make_pair_sequence(sequence, seq::lift(value));
}

template <typename Sequence>
auto operator+(const typename flatten_sequence<Sequence>::value_type &value, const flatten_sequence<Sequence> &sequence) {
    return seq::make_pair_sequence(seq::lift(value), sequence);
}



template <typename Sequence>
std::ostream &operator<<(std::ostream &stream, const flatten_sequence<Sequence> &sequence) {
    auto lambda = [&stream](const auto &value) {
        stream << value;
    };

    seq::for_each(sequence, lambda);

    return stream;
}



template <typename Sequence>
auto flatten_sequence<Sequence>::operator*() const -> std::optional<value_type> {
    auto lambda = [](const auto &value) {
        return *value;
    };

    return utl::opt::and_then(lambda, m_subsequence);
}

template <typename Sequence>
flatten_sequence<Sequence> &flatten_sequence<Sequence>::operator++() {
    auto lambda = [this](auto &&sequence) {
        ++sequence;

        if(!*sequence) {
            ++m_sequence;

            auto lambda = [](auto &&sequence) {
                return seq::size(std::forward<decltype(sequence)>(sequence)) != 0u;
            };

            auto index = seq::find_if(m_sequence, lambda);

            if(index != seq::size(m_sequence)) {
                seq::advance(m_sequence, index);

                m_subsequence.emplace(seq::front(m_sequence));
            }
        }

        return nullptr;
    };

    utl::opt::and_then(lambda, m_subsequence);

    return *this;
}

template <typename Sequence>
flatten_sequence<Sequence> flatten_sequence<Sequence>::operator++(int) {
    auto auxiliar = *this;

    operator++();

    return auxiliar;
}



template <typename Sequence>
flatten_sequence<std::decay_t<Sequence>> flatten(Sequence &&sequence) {
    return seq::flatten_sequence<std::decay_t<Sequence>>(utl::forward, std::forward<Sequence>(sequence));
}

}

#endif
