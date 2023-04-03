
#ifndef _PMR_HEADERS_HPP_
#define _PMR_HEADERS_HPP_


#include <string>
#include <vector>
#include <list>
#include <forward_list>
#include <map>
#include <unordered_map>
#include <deque>
#include <set>
#include <unordered_set>
#include "polymorphic_allocator.hpp"


namespace pmr {

    // C++17 string that uses a polymorphic allocator
    template <class charT, class traits = std::char_traits<charT>>
    using basic_string = std::basic_string<charT, traits,
                                           polymorphic_allocator<charT>>;

    using string  = basic_string<char>;
    using wstring = basic_string<wchar_t>;


    // C++17 vector container that uses a polymorphic allocator
    template <class Tp>
    using vector = std::vector<Tp, polymorphic_allocator<Tp>>;

    // C++17 list using pmr
    template <class T>
    using list = std::list<T, pmr::polymorphic_allocator<T>>;

    // C++17 forward_list container that uses a polymorphic allocator
    template <class T>
    using forward_list = std::forward_list<T, polymorphic_allocator<T>>;


    // C++17 map container that uses a polymorphic allocator
    template<
        class Key,
        class T,
        class Compare = std::less<Key>
    > using map = std::map<Key, T, Compare,
                        polymorphic_allocator<std::pair<const Key, T>>>;

    // C++17 multimap container that uses a polymorphic allocator
    template<
        class Key,
        class T,
        class Compare = std::less<Key>
    > using multimap =
        std::multimap<Key, T, Compare,
                    polymorphic_allocator<std::pair<const Key, T>>>;

    // C++17 unordered_map container that uses a polymorphic allocator
    template <
        class Key,
        class T,
        class Hash = std::hash<Key>,
        class KeyEqual = std::equal_to<Key>
    > using unordered_map = std::unordered_map<Key, T, Hash, KeyEqual,
                                polymorphic_allocator<std::pair<const Key,T>>>;

    // C++17 unordered_multimap container that uses a polymorphic allocator
    template<
        class Key, class T,
        class Hash = std::hash<Key>,
        class Pred = std::equal_to<Key>
    > using unordered_multimap = std::unordered_multimap<Key, T, Hash, Pred,
                                    polymorphic_allocator<std::pair<const Key,T>>>;


    // C++17 deque container that uses a polymorphic allocator
    template< class T >
        using deque = std::deque<T, polymorphic_allocator<T>>;


    // C++17 set container that uses a polymorphic allocator
    template<
        class Key,
        class Compare = std::less<Key>
    > using set = std::set<Key, Compare, polymorphic_allocator<Key>>;

    // C++17 multiset_set container that uses a polymorphic allocator
    template<
        class Key,
        class Compare = std::less<Key>
    > using multiset = std::multiset<Key, Compare, polymorphic_allocator<Key>>;

    // C++17 unordered_set container that uses a polymorphic allocator
    template< class Key,
            class Hash = std::hash<Key>,
            class Pred = std::equal_to<Key> >
    using unordered_set = std::unordered_set<Key, Hash, Pred,
                                            polymorphic_allocator<Key>>;

    // C++17 unordered_multiset container that uses a polymorphic allocator
    template<
        class Key,
        class Hash = std::hash<Key>,
        class Pred = std::equal_to<Key>
    > using unordered_multiset =
        std::unordered_multiset<Key, Hash, Pred, polymorphic_allocator<Key>>;

}

#endif // pmr_headers.hpp


/********************************** End of File *******************************/