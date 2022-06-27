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

#if !defined(SEQUENCE_ALGORITHM_HEADER_IS_INCLUDED)
#define SEQUENCE_ALGORITHM_HEADER_IS_INCLUDED

#include "sequence/variant_sequence.hpp"

namespace seq {

template <typename Natural>
auto make_numeric_sequence() {
    auto lambda_0 = [] {
        return std::make_optional<Natural>(Natural{ });
    };

    auto lambda_1 = [](auto natural) {
        return std::make_optional<Natural>(++natural);
    };

    return seq::make_while_sequence(lambda_0, lambda_1);
}

template <typename Natural>
auto make_numeric_sequence(Natural natural) {
    auto lambda_0 = [natural] {
        return natural ? std::make_optional<Natural>() : std::optional<Natural>();
    };

    auto lambda_1 = [natural](auto natural_) {
        return ++natural_ < natural ? std::make_optional<Natural>(natural_) : std::optional<Natural>();
    };

    return seq::make_while_sequence(lambda_0, lambda_1);
}



template <typename Sequence>
auto reverse(Sequence &&sequence) {
    auto size = seq::size(sequence);

    auto lambda = [sequence = std::forward<Sequence>(sequence), size](auto index) {
        return seq::at(std::move(sequence), size - index - 1u);
    };

    return seq::transform(std::move(lambda), seq::make_numeric_sequence(size));
}



template <typename Sequence>
std::optional<typename std::decay_t<Sequence>::value_type> last(Sequence &&sequence) {
    return *seq::reverse(std::forward<Sequence>(sequence));
}

template <typename Sequence>
typename std::decay_t<Sequence>::value_type back(Sequence &&sequence) {
    return seq::last(std::forward<Sequence>(sequence)).value();
}



template <typename Callable, typename... Sequence>
auto and_then(Callable &&callable, Sequence &&... sequence) {
    return seq::flatten(seq::transform(std::forward<Callable>(callable), std::forward<Sequence>(sequence)...));
}



template <typename Sequence>
auto continuum(Sequence &&sequence) {
    auto next = seq::next(sequence);

    auto lambda = [](const auto &value_0, const auto &value_1) {
        return typename std::decay_t<Sequence>::continuum_type(value_0, value_1);
    };

    return seq::transform(lambda, std::forward<Sequence>(sequence), std::move(next));
}



inline auto advance(continuous_size size_0, continuous_size size_1) {
    return continuous_size(size_0.first + size_1.first, size_0.first == 0 ? (size_1.second + (1 - size_1.second) * size_0.second) : size_0.second);
}

inline auto advance(const continuous_subsequence &subsequence, continuous_size size) {
    return continuous_subsequence(seq::advance(subsequence.first, size), seq::advance(subsequence.second, size));
}



inline subsequence pit(const continuous_subsequence &thesubsequence) {
    return subsequence(thesubsequence.first.first + 1u, thesubsequence.second.first + 1u);
}



template <typename Sequence>
auto limit(Sequence &&sequence, std::size_t size) {
    auto lambda = [](const auto &value, auto) {
        return value;
    };

    return seq::transform(lambda, std::forward<Sequence>(sequence), seq::make_numeric_sequence(size));
}



template <typename Sequence>
auto make_subsequebce_sequence(Sequence &&sequence, subsequence subsequence) {
    return seq::limit(seq::next(std::forward<Sequence>(sequence), subsequence.first), subsequence.second - subsequence.first);
}

template <typename Sequence>
auto make_subsequebce_sequence(Sequence &&sequence, const continuous_subsequence &subsequence) {
    auto front = seq::at(sequence, subsequence.first);

    auto back = seq::at(sequence, subsequence.second);

    return std::move(front) + seq::make_subsequebce_sequence(std::forward<Sequence>(sequence), seq::pit(subsequence)) + std::move(back);
}



template <typename Sequence>
typename std::decay_t<Sequence>::value_type at(Sequence &&sequence, endpoint point) {
    switch(point) {
        case endpoint::front: {
            return seq::front(sequence);
        }

        case endpoint::back: {
            return seq::back(std::forward<Sequence>(sequence));
        }

        default: {
            throw std::logic_error(__func__);
        }
    }
}

template <typename Bisequence>
auto at(const Bisequence &bichain, biendpoint endpoint) {
    switch(endpoint) {
        case biendpoint::front_first: {
            return seq::front(bichain).first;
        }

        case biendpoint::front_second: {
            return seq::front(bichain).second;
        }

        case biendpoint::back_first: {
            return seq::back(bichain).first;
        }

        case biendpoint::back_second: {
            return seq::back(bichain).second;
        }

        default: {
            throw std::logic_error(__func__);
        }
    }
}



template <typename Sequence>
auto make_index_sequence(Sequence &&sequence) {
    return seq::make_numeric_sequence(seq::size(std::forward<Sequence>(sequence)));
}

template <typename Enumeration>
auto make_enum_sequence() {
    auto lambda_0 = []() {
        return std::make_optional<Enumeration>(Enumeration{ });
    };

    auto lambda_1 = [](auto enumeration) {
        return ++enumeration != Enumeration{ } ? std::make_optional<Enumeration>(enumeration) : std::optional<Enumeration>();
    };

    return seq::make_while_sequence(lambda_0, lambda_1);
}



template <typename Value>
auto empty() {
    auto lambda = [] {
        return std::optional<Value>();
    };

    return seq::make_while_sequence_(lambda);
}



template <typename Generator0, typename Generator1>
variant_sequence<utl::call_t<Generator0>, utl::call_t<Generator1>> conditional(bool boolean, Generator0 &&generator_0, Generator1 &&generator_1) {
    using return_type = variant_sequence<utl::call_t<Generator0>, utl::call_t<Generator1>>;

    return boolean ? return_type(utl::forward, generator_0()) : return_type(utl::forward, generator_1());
}



template <typename Matrix, typename Sequence>
auto deref(Matrix &&matrix, Sequence &&sequence) {
    auto lambda = [matrix = matrix](auto i, auto j) {
        return seq::at(seq::at(matrix, i), j);
    };

    return seq::transform(std::move(lambda), seq::make_index_sequence(std::forward<Matrix>(matrix)), std::forward<Sequence>(sequence));
}

template <typename Sequence0, typename Sequence1>
auto arrange(Sequence0 &&sequence_0, Sequence1 &&sequence_1) {
    auto lambda = [sequence_0 = std::forward<Sequence0>(sequence_0)](auto index) {
        return seq::at(sequence_0, index);
    };

    return seq::transform(std::move(lambda), std::forward<Sequence1>(sequence_1));
}

template <typename Sequence>
auto indexed(Sequence &&sequence) {
    auto lambda = [](auto index, auto &&value) {
        return std::pair(index, std::move(value));
    };

    auto index_sequence = seq::make_index_sequence(sequence);

    return seq::transform(lambda, index_sequence, std::forward<Sequence>(sequence));
}



template <typename Sequence, typename Callable>
auto filter(Sequence &&sequence, Callable &&callable) {
    auto lambda = [callable = std::forward<Callable>(callable)](auto &&value) {
        auto boolean = callable(value);

        auto lambda_0 = [value = std::move(value)] {
            return seq::lift(std::move(value));
        };

        auto lambda_1 = [] {
            return seq::empty<typename std::decay_t<Sequence>::value_type>();
        };

        return seq::conditional(boolean, std::move(lambda_0), lambda_1);
    };

    return seq::and_then(std::move(lambda), std::forward<Sequence>(sequence));
}

template <typename Break, typename Continue, typename Callable, typename... Sequence>
auto iterate(Callable &&callable, Continue &&initial, Sequence &&... sequence) {
    using value_type = std::variant<Break, std::decay_t<Continue>>;

    auto lambda_0 = [callable, initial_ = std::forward<Continue>(initial)](auto &&... value) -> std::optional<value_type> {
        return callable(std::move(initial_), std::move(value)...);
    };

    auto lambda_1 = [callable](auto &&variant, auto &&... value) -> std::optional<value_type> {
        auto lambda = [callable, &value...](const auto &current) -> std::optional<value_type> {
            if constexpr(std::is_same_v<std::decay_t<decltype(current)>, Break>) {
                return { };
            } else {
                return callable(current, value...);
            }
        };

        return std::visit(lambda, variant);
    };

    return seq::make_while_sequence(lambda_0, lambda_1, std::forward<Sequence>(sequence)...);
}



template <typename Sequence>
std::size_t min(Sequence &&sequence) {
    auto lambda = [](auto &&pair_0, auto &&pair_1) {
        return pair_1.second < pair_0.second ? std::forward<decltype(pair_1)>(pair_1) : std::forward<decltype(pair_0)>(pair_0);
    };

    auto pair_sequence = seq::indexed(std::forward<Sequence>(sequence));

    auto front = seq::front(pair_sequence);

    return seq::accumulate(seq::next(std::move(pair_sequence)), std::move(front), lambda).first;
}

template <typename Sequence>
typename Sequence::value_type min(as_value_t, Sequence &&sequence) {
    auto index = seq::min(sequence);

    return seq::at(std::forward<Sequence>(sequence), index);
}

template <typename Sequence>
std::size_t max(Sequence &&sequence) {
    auto lambda = [](auto &&pair_0, auto &&pair_1) {
        return pair_1.second > pair_0.second ? std::forward<decltype(pair_1)>(pair_1) : std::forward<decltype(pair_0)>(pair_0);
    };

    auto pair_sequence = seq::indexed(std::forward<Sequence>(sequence));

    auto front = seq::front(pair_sequence);

    return seq::accumulate(seq::next(std::move(pair_sequence)), std::move(front), lambda).first;
}

template <typename Sequence>
typename Sequence::value_type max(as_value_t, Sequence &&sequence) {
    auto index = seq::max(sequence);

    return seq::at(std::forward<Sequence>(sequence), index);
}



template <typename Sequence>
auto alternate(Sequence &&sequence) {
    auto lambda = [](const auto &pair, auto index) {
        return utl::even(index) ? pair : utl::permute(pair);
    };

    auto index_sequence = seq::make_index_sequence(sequence);

    return seq::transform(lambda, std::forward<Sequence>(sequence), std::move(index_sequence));
}

template <typename Sequence0, typename Sequence1, typename Callable>
auto confront(Sequence0 &&sequence_0, Sequence1 &&sequence_1, Callable &&callable) {
    auto lambda = [sequence_1 = std::forward<Sequence1>(sequence_1), callable = std::forward<Callable>(callable)](auto &&value_0) {
        auto lambda = [value_0 = std::forward<decltype(value_0)>(value_0), callable](auto &&value_1) {
            return callable(value_0, std::forward<decltype(value_1)>(value_1));
        };

        return seq::make_container_sequence_(seq::transform(std::move(lambda), std::move(sequence_1)));
    };

    return seq::transform(std::move(lambda), std::forward<Sequence0>(sequence_0));
}

template <typename Sequence0, typename Sequence1>
auto intersperse(Sequence0 &&sequence_0, Sequence1 &&sequence_1) {
    auto nonempty = static_cast<bool>(*sequence_0);

    auto lambda_0 = [sequence_0 = std::forward<Sequence0>(sequence_0), sequence_1 = std::forward<Sequence1>(sequence_1)] {
        auto lambda = [](auto &&value_0, auto &&value_1) {
            return seq::lift(std::forward<decltype(value_0)>(value_0)) + std::forward<decltype(value_1)>(value_1);
        };

        return seq::front(sequence_0) + seq::and_then(lambda, sequence_1, ++Sequence0(sequence_0));
    };

    auto lambda_1 = [] {
        return seq::empty<typename decltype(lambda_0())::value_type>();
    };

    return seq::conditional(nonempty, lambda_0, lambda_1);
}

template <typename Sequence>
auto sort(Sequence &&sequence) {
    auto vector_0 = seq::accumulate<std::vector<std::pair<std::size_t, typename std::decay_t<Sequence>::value_type>>>(seq::indexed(std::forward<Sequence>(sequence)));

    auto lambda_0 = [](const auto &pair_0, const auto &pair_1) {
        return pair_0.second < pair_1.second;
    };

    std::sort(vector_0.begin(), vector_0.end(), lambda_0);

    auto lambda_1 = [](const auto &pair) {
        return pair.first;
    };

    auto vector_1 = seq::accumulate<std::vector<std::pair<std::size_t, std::size_t>>>(seq::indexed(seq::transform(lambda_1, seq::make_container_sequence(std::move(vector_0)))));

    auto lambda_2 = [](const auto &pair_0, const auto &pair_1) {
        return pair_0.second < pair_1.second;
    };

    std::sort(vector_1.begin(), vector_1.end(), lambda_2);

    std::vector<std::size_t> indices;

    indices.reserve(vector_1.size());

    for(const auto &pair : vector_1) {
        indices.push_back(pair.first);
    }

    return seq::make_container_sequence(std::move(indices));
}

template <typename Bisequence>
auto permute(Bisequence &&bisequence) {
    auto lambda = [](auto &&pair) {
        return utl::permute(std::move(pair));
    };

    return seq::transform(lambda, std::forward<Bisequence>(bisequence));
}

template <typename Sequence>
auto nonrepeated(Sequence &&sequence) {
    auto lambda = [](const auto &value_0, auto &&value_1) {
        auto lambda_0 = [value_1_ = std::forward<decltype(value_1)>(value_1)] {
            return seq::lift(value_1_);
        };

        auto lambda_1 = [] {
            return seq::empty<typename std::decay_t<Sequence>::value_type>();
        };

        return seq::conditional(value_0 != value_1, std::move(lambda_0), lambda_1);
    };

    auto front = seq::front(sequence);

    auto next = seq::next(sequence);

    return std::move(front) + seq::and_then(lambda, std::forward<Sequence>(sequence), std::move(next));
};



template <endpoint Endpoint, typename Sequence, std::enable_if_t<Endpoint == endpoint::front, bool> = true>
std::decay_t<Sequence> plop(Sequence &&sequence) {
    return std::forward<Sequence>(sequence);
}

template <endpoint Endpoint, typename Sequence, std::enable_if_t<Endpoint == endpoint::back, bool> = true>
auto plop(Sequence &&sequence) {
    return seq::reverse(std::forward<Sequence>(sequence));
}

template <typename Sequence>
auto plop(Sequence &&sequence, endpoint point) {
    using front_type = decltype(seq::plop<endpoint::front>(std::forward<Sequence>(sequence)));
    using back_type = decltype(seq::plop<endpoint::back>(std::forward<Sequence>(sequence)));
    using return_type = seq::variant_sequence<front_type, back_type>;

    switch(point) {
        case endpoint::front: {
            return return_type(utl::forward, seq::plop<endpoint::front>(std::forward<Sequence>(sequence)));
        }

        case endpoint::back: {
            return return_type(utl::forward, seq::plop<endpoint::back>(std::forward<Sequence>(sequence)));
        }

        default: {
            throw std::logic_error(__func__);
        }
    }
}

template <biendpoint Biendpoint, typename Bisequence, std::enable_if_t<Biendpoint == biendpoint::front_first, bool> = true>
auto plop(Bisequence &&bisequence) {
    auto lambda = [](const auto &pair) {
        return seq::lift(pair.first) + pair.second;
    };

    return seq::and_then(lambda, std::forward<Bisequence>(bisequence));
}

template <biendpoint Biendpoint, typename Bisequence, std::enable_if_t<Biendpoint == biendpoint::front_second, bool> = true>
auto plop(Bisequence &&bisequence) {
    return seq::plop<biendpoint::front_first>(seq::permute(std::forward<Bisequence>(bisequence)));
}

template <biendpoint Biendpoint, typename Bisequence, std::enable_if_t<Biendpoint == biendpoint::back_first, bool> = true>
auto plop(Bisequence &&bisequence) {
    return seq::plop<biendpoint::front_first>(seq::reverse(std::forward<Bisequence>(bisequence)));
}

template <biendpoint Biendpoint, typename Bisequence, std::enable_if_t<Biendpoint == biendpoint::back_second, bool> = true>
auto plop(Bisequence &&bisequence) {
    return seq::plop<biendpoint::front_second>(seq::reverse(std::forward<Bisequence>(bisequence)));
}

template <typename Bisequence>
auto plop(Bisequence &&bisequence, biendpoint endpoint) {
    using front_first_type = decltype(plop<biendpoint::front_first>(std::forward<Bisequence>(bisequence)));
    using front_second_type = decltype(plop<biendpoint::front_second>(std::forward<Bisequence>(bisequence)));
    using back_first_type = decltype(plop<biendpoint::back_first>(std::forward<Bisequence>(bisequence)));
    using back_second_type = decltype(plop<biendpoint::back_second>(std::forward<Bisequence>(bisequence)));

    using return_type = seq::variant_sequence<front_first_type, front_second_type, back_first_type, back_second_type>;

    switch(endpoint) {
        case biendpoint::front_first: {
            return return_type(utl::forward, seq::plop<biendpoint::front_first>(std::forward<Bisequence>(bisequence)));
        }

        case biendpoint::front_second: {
            return return_type(utl::forward, seq::plop<biendpoint::front_second>(std::forward<Bisequence>(bisequence)));
        }

        case biendpoint::back_first: {
            return return_type(utl::forward, seq::plop<biendpoint::back_first>(std::forward<Bisequence>(bisequence)));
        }

        case biendpoint::back_second: {
            return return_type(utl::forward, seq::plop<biendpoint::back_second>(std::forward<Bisequence>(bisequence)));
        }

        default: {
            throw std::logic_error(__func__);
        }
    }
}

}

#endif
