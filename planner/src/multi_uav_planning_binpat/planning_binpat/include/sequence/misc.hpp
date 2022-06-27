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
 * "AS IS" AND ANY ETypePRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * ETypeEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#if !defined(SEQUENCE_MISC_HEADER_IS_INCLUDED)
#define SEQUENCE_MISC_HEADER_IS_INCLUDED

#include <stdexcept>

#include "utility.hpp"

namespace seq {

struct as_sequence_t { };

inline constexpr as_sequence_t as_sequence;


struct as_value_t { };

inline constexpr as_value_t as_value;



using subsequence = std::pair<std::size_t, std::size_t>;



using continuous_size = std::pair<std::size_t, utl::percentage>;

using continuous_subsequence = std::pair<continuous_size, continuous_size>;



enum class endpoint {
    front,
    back,
};

inline endpoint &operator++(endpoint &point) {
    switch(point) {
        case endpoint::front: {
            return point = endpoint::back;
        }

        case endpoint::back: {
            return point = endpoint::front;
        }

        default: {
            throw std::logic_error(__func__);
        }
    }
}



enum biendpoint {
    front_first,
    front_second,
    back_first,
    back_second,
};

inline biendpoint &operator++(biendpoint &point) {
    switch(point) {
        case biendpoint::front_first: {
            return point = biendpoint::front_second;
        }

        case biendpoint::front_second: {
            return point = biendpoint::back_first;
        }

        case biendpoint::back_first: {
            return point = biendpoint::back_second;
        }

        case biendpoint::back_second: {
            return point = biendpoint::front_first;
        }

        default: {
            throw std::logic_error(__func__);
        }
    }
}



template <typename Sequence, typename Callable>
void for_each(Sequence sequence, Callable &&callable) {
    while(auto value = *sequence) {
        callable(*std::move(value));

        ++sequence;
    }
}



template <typename Sequence, typename Result, typename Callable>
Result accumulate(Sequence sequence, Result result, Callable &&callable) {
    while(auto value = *sequence) {
        result = callable(std::move(result), *std::move(value));

        ++sequence;
    }

    return result;
}

template <typename Sequence>
typename std::decay_t<Sequence>::value_type accumulate(Sequence &&sequence) {
    auto lambda = [](const auto &result, const auto &value) {
        return result + value;
    };

    return seq::accumulate(std::forward<Sequence>(sequence), typename std::decay_t<Sequence>::value_type{ }, lambda);
}

template <typename Container, typename Sequence>
Container accumulate(Sequence &&sequence) {
    auto lambda = [](auto &&container, auto &&value) -> Container {
        container.emplace_back(std::forward<decltype(value)>(value));

        return std::forward<decltype(container)>(container);
    };

    return seq::accumulate(std::forward<Sequence>(sequence), Container(), lambda);
}



template <typename Sequence, typename Predicate>
std::size_t find_if(Sequence sequence, Predicate &&predicate) {
    std::size_t index{ };

    while(auto value = *sequence) {
        if(predicate(*std::move(value))) {
            return index;
        }

        ++index, ++sequence;
    }

    return index;
}

template <typename Sequence, typename Callable>
std::optional<std::pair<std::size_t, typename utl::call_t<Callable, typename Sequence::value_type>::value_type>> find(Sequence sequence, Callable &&callable) {
    std::size_t index{ };

    while(auto value = *sequence) {
        if(auto result = callable(*std::move(value))) {
            return std::pair(index, *std::move(result));
        }

        ++index, ++sequence;
    }

    return { };
}



template <typename Sequence>
std::size_t raw_size(Sequence &&sequence) {
    auto lambda = [](auto size, const auto &) {
        return ++size;
    };

    return seq::accumulate(std::forward<Sequence>(sequence), 0u, lambda);
}

template <typename Sequence>
std::size_t size(const Sequence &sequence) {
    return sequence.size();
}



template <typename Sequence>
void advance(Sequence &sequence, std::size_t size) {
    while(size) {
        ++sequence, --size;
    }
}

template <typename Sequence>
Sequence next(Sequence sequence, std::size_t size = 1u) {
    seq::advance(sequence, size);

    return sequence;
}



template <typename Sequence>
std::optional<typename Sequence::value_type> first(const Sequence &sequence) {
    return *sequence;
}

template <typename Sequence>
typename Sequence::value_type front(const Sequence &sequence) {
    return seq::first(sequence).value();
}



template <typename Sequence>
bool empty(const Sequence &sequence) {
    return !seq::first(sequence).has_value();
}



template <typename Sequence>
typename std::decay_t<Sequence>::value_type at(Sequence &&sequence, std::size_t index) {
    return seq::front(seq::next(std::forward<Sequence>(sequence), index));
}



template <typename Sequence>
auto at(Sequence &&sequence, continuous_size size) {
    auto first = seq::at(sequence, size.first);

    auto second = seq::at(std::forward<Sequence>(sequence), size.first + 1u);

    return at(typename std::decay_t<Sequence>::continuum_type(first, second), size.second);
}

template <typename Sequence>
auto next(Sequence &&sequence, continuous_size size) {
    auto value = seq::at(sequence, size);

    return value + seq::next(std::forward<Sequence>(sequence), size.first + 1u);
}

}

#endif
