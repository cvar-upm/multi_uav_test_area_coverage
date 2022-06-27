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

#if !defined(SEQUENCE_WHILE_SEQUENCE_HEADER_IS_INCLUDED)
#define SEQUENCE_WHILE_SEQUENCE_HEADER_IS_INCLUDED

#include <ostream>

#include "utility.hpp"

#include "sequence/misc.hpp"

namespace seq {

template <typename Callable0, typename Callable1, typename... Sequence>
class while_sequence {
public:
    using callable_0_type = Callable0;
    using callable_1_type = Callable1;
    using value_type = typename utl::call_t<callable_0_type, typename Sequence::value_type...>::value_type;

    template <typename Callable0Forward, typename Callable1Forward, typename... SequenceForward>
    while_sequence(Callable0Forward &&callable_0, Callable1Forward &&callable_1, SequenceForward &&... sequence) : m_callable_0(std::forward<Callable0Forward>(callable_0)), m_callable_1(std::forward<Callable1Forward>(callable_1)), m_tuple(std::forward<SequenceForward>(sequence)...) {
        auto lambda = [this](auto &... sequence) {
            return utl::opt::and_then(m_callable_0, *sequence++...);
        };

        if(auto optional = std::apply(lambda, m_tuple)) {
            m_value.emplace(*std::move(optional));
        }
    }

    std::optional<value_type> operator*() const { return m_value; }

    while_sequence &operator++();

    while_sequence operator++(int);

    std::size_t size() const { return seq::raw_size(*this); }

private:
    callable_0_type m_callable_0;
    callable_1_type m_callable_1;
    std::tuple<Sequence...> m_tuple;
    std::optional<value_type> m_value;
};



template <typename Callable0, typename Callable1, typename... Sequence>
std::ostream &operator<<(std::ostream &stream, const while_sequence<Callable0, Callable1, Sequence...> &sequence) {
    auto lambda = [&stream](const auto &value) {
        stream << value;
    };

    seq::for_each(sequence, lambda);

    return stream;
}



template <typename Callable0, typename Callable1, typename... Sequence>
while_sequence<Callable0, Callable1, Sequence...> &while_sequence<Callable0, Callable1, Sequence...>::operator++() {
    auto lambda = [this](const auto &value) {
        auto lambda = [this, &value](auto &... sequence) {
            auto lambda = [this, &value](auto &&... values) -> std::optional<value_type> {
                return m_callable_1(std::move(value), std::forward<decltype(values)>(values)...);
            };

            return utl::opt::and_then(lambda, *sequence++...);
        };

        return std::apply(lambda, m_tuple);
    };

    if(auto optional = utl::opt::and_then(lambda, m_value)) {
        m_value.emplace(*std::move(optional));
    } else {
        m_value.reset();
    }

    return *this;
}

template <typename Callable0, typename Callable1, typename... Sequence>
while_sequence<Callable0, Callable1, Sequence...> while_sequence<Callable0, Callable1, Sequence...>::operator++(int) {
    auto auxiliar = *this;

    operator++();

    return auxiliar;
}



template <typename Callable0, typename Callable1, typename... Sequence>
while_sequence<std::decay_t<Callable0>, std::decay_t<Callable1>, std::decay_t<Sequence>...> make_while_sequence(Callable0 &&callable_0, Callable1 &&callable_1, Sequence &&... sequence) {
    return while_sequence<std::decay_t<Callable0>, std::decay_t<Callable1>, std::decay_t<Sequence>...>(std::forward<Callable0>(callable_0), std::forward<Callable1>(callable_1), std::forward<Sequence>(sequence)...);
}

template <typename Callable, typename... Sequence>
auto make_while_sequence_(Callable &&callable, Sequence &&... sequence) {
    auto lambda = [callable = std::forward<Callable>(callable)](const auto &, auto &&... value) {
        return callable(std::forward<decltype(value)>(value)...);
    };

    return while_sequence<std::decay_t<Callable>, decltype(lambda), std::decay_t<Sequence>...>(std::forward<Callable>(callable), std::move(lambda), std::forward<Sequence>(sequence)...);
}



template <typename Value>
auto lift(Value &&value) {
    auto lambda_0 = [value = std::forward<Value>(value)] {
        return std::make_optional(std::move(value));
    };

    auto lambda_1 = [](const auto &) {
        return std::optional<std::decay_t<Value>>();
    };

    return seq::make_while_sequence(std::move(lambda_0), lambda_1);
}

}

#endif
