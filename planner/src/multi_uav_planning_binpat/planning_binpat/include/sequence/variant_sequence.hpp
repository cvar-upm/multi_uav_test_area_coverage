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

#if !defined(SEQUENCE_VARIANT_SEQUENCE_HEADER_IS_INCLUDED)
#define SEQUENCE_VARIANT_SEQUENCE_HEADER_IS_INCLUDED

#include <functional>

#include "sequence/pair_sequence.hpp"

namespace seq {

template <typename... Sequence>
class variant_sequence {
public:
    using value_type = std::common_type_t<typename Sequence::value_type...>;

    template <typename VariantForward>
    variant_sequence(utl::forward_t, VariantForward &&variant) : m_variant(std::forward<VariantForward>(variant)) {  }

    std::optional<value_type> operator*() const;

    variant_sequence &operator++();

    variant_sequence operator++(int);

    std::size_t size() const {
        auto lambda = [](const auto &sequence) {
            return sequence.size();
        };

        return std::visit(lambda, m_variant);
    }

private:
    std::variant<Sequence...> m_variant;
};



template <typename... Sequence>
auto operator+(const variant_sequence<Sequence...> &sequence, const typename variant_sequence<Sequence...>::value_type &value) {
    return seq::make_pair_sequence(sequence, seq::lift(value));
}

template <typename... Sequence>
auto operator+(const typename variant_sequence<Sequence...>::value_type &value, const variant_sequence<Sequence...> &sequence) {
    return seq::make_pair_sequence(seq::lift(value), sequence);
}



template <typename... Sequence>
std::ostream &operator<<(std::ostream &stream, const variant_sequence<Sequence...> &sequence) {
    auto lambda = [&stream](const auto &value) {
        stream << value;
    };

    seq::for_each(sequence, lambda);

    return stream;
}



template <typename... Sequence>
auto variant_sequence<Sequence...>::operator*() const -> std::optional<value_type> {
    auto lambda = [](const auto &sequence) {
        auto lambda = [](auto &&value) {
            return value_type(std::forward<decltype(value)>(value));
        };

        return utl::opt::transform(lambda, *sequence);
    };

    return std::visit(lambda, m_variant);
}

template <typename... Sequence>
variant_sequence<Sequence...> &variant_sequence<Sequence...>::operator++() {
    auto lambda = [](auto &sequence) {
        ++sequence;
    };

    std::visit(lambda, m_variant);

    return *this;
}

template <typename... Sequence>
variant_sequence<Sequence...> variant_sequence<Sequence...>::operator++(int) {
    auto sequence = *this;

    operator++();

    return sequence;
}

}

#endif
