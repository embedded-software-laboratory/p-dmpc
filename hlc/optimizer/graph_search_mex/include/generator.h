#ifndef INCLUDED_CORO_UNIQUE_GENERATOR_H
#define INCLUDED_CORO_UNIQUE_GENERATOR_H

// Original source:
// https://github.com/lewissbaker/llvm/blob/9f59dcce/coroutine_examples/manual_lifetime.hpp
// https://github.com/lewissbaker/llvm/blob/9f59dcce/coroutine_examples/generator.hpp

#if __has_include(<coroutine>)
#include <coroutine>
#else
#include <experimental/coroutine>
namespace std {
	using std::experimental::coroutine_handle;
	using std::experimental::noop_coroutine;
	using std::experimental::suspend_always;
	using std::experimental::suspend_never;
}  // namespace std
#endif  // __has_include(<coroutine>)

#include <iterator>
#include <memory>
#include <utility>

#ifndef INCLUDED_CORO_MANUAL_LIFETIME_H
#define INCLUDED_CORO_MANUAL_LIFETIME_H

template <class T>
struct manual_lifetime {
   public:
	manual_lifetime() noexcept {}
	~manual_lifetime() noexcept {}

	template <class... Args>
	void construct(Args&&... args) {
		::new (static_cast<void*>(std::addressof(value))) T(static_cast<Args&&>(args)...);
	}

	void destruct() noexcept { value.~T(); }

	T& get() & { return value; }
	const T& get() const& { return value; }
	T&& get() && { return (T &&) value; }
	const T&& get() const&& { return (const T&&)value; }

   private:
	union {
		T value;
	};
};

template <class T>
struct manual_lifetime<T&> {
	manual_lifetime() noexcept = default;

	void construct(T& value) noexcept { ptr = std::addressof(value); }
	void destruct() noexcept {}
	T& get() const noexcept { return *ptr; }

   private:
	T* ptr = nullptr;
};

template <class T>
struct manual_lifetime<T&&> {
	manual_lifetime() noexcept = default;

	void construct(T&& value) noexcept { ptr = std::addressof(value); }
	void destruct() noexcept {}
	T&& get() const noexcept { return *ptr; }

   private:
	T* ptr = nullptr;
};

template <>
struct manual_lifetime<void> {
	void construct() noexcept {}
	void destruct() noexcept {}
	void get() const noexcept {}
};

#endif  // INCLUDED_CORO_MANUAL_LIFETIME_H

template <class Ref, class Value = std::decay_t<Ref>>
class unique_generator {
   public:
	class promise_type {
	   public:
		promise_type() noexcept = default;

		~promise_type() noexcept { clear_value(); }

		void clear_value() {
			if (hasValue_) {
				hasValue_ = false;
				ref_.destruct();
			}
		}

		unique_generator get_return_object() noexcept { return unique_generator(std::coroutine_handle<promise_type>::from_promise(*this)); }

		auto initial_suspend() noexcept { return std::suspend_always{}; }

		auto final_suspend() noexcept { return std::suspend_always{}; }

		auto yield_value(Ref ref) noexcept(std::is_nothrow_move_constructible_v<Ref>) {
			ref_.construct(std::move(ref));
			hasValue_ = true;
			return std::suspend_always{};
		}

		void return_void() {}

		void unhandled_exception() { throw; }

		Ref get() { return ref_.get(); }

	   private:
		manual_lifetime<Ref> ref_;
		bool hasValue_ = false;
	};

	using handle_t = std::coroutine_handle<promise_type>;

	unique_generator(unique_generator&& g) noexcept : coro_(std::exchange(g.coro_, {})) {}

	~unique_generator() {
		if (coro_) {
			coro_.destroy();
		}
	}

	struct sentinel {};

	class iterator {
	   public:
		using reference = Ref;
		using value_type = Value;
		using difference_type = std::ptrdiff_t;
		using pointer = std::add_pointer_t<Ref>;
		using iterator_category = std::input_iterator_tag;

		iterator() noexcept {}

		explicit iterator(handle_t coro) noexcept : coro_(coro) {}

		reference operator*() const { return coro_.promise().get(); }

		iterator& operator++() {
			coro_.promise().clear_value();
			coro_.resume();
			return *this;
		}

		void operator++(int) {
			coro_.promise().clear_value();
			coro_.resume();
		}

		friend bool operator==(const iterator& it, sentinel) noexcept { return it.coro_.done(); }
		friend bool operator==(sentinel, const iterator& it) noexcept { return it.coro_.done(); }
		friend bool operator!=(const iterator& it, sentinel) noexcept { return !it.coro_.done(); }
		friend bool operator!=(sentinel, const iterator& it) noexcept { return !it.coro_.done(); }

	   private:
		handle_t coro_;
	};

	iterator begin() {
		coro_.resume();
		return iterator{coro_};
	}

	sentinel end() { return {}; }

   private:
	explicit unique_generator(handle_t coro) noexcept : coro_(coro) {}

	handle_t coro_;
};

#endif  // INCLUDED_CORO_UNIQUE_GENERATOR_H

/*
#ifndef INCLUDED_CORO_SHARED_GENERATOR_H
#define INCLUDED_CORO_SHARED_GENERATOR_H

// Original source:
// https://github.com/lewissbaker/llvm/blob/9f59dcce/coroutine_examples/manual_lifetime.hpp
// https://github.com/lewissbaker/llvm/blob/9f59dcce/coroutine_examples/generator.hpp
// https://github.com/ericniebler/range-v3/blob/664aa80/include/range/v3/experimental/utility/generator.hpp

#if __has_include(<coroutine>)

#include <coroutine>

#else
#include <experimental/coroutine>
namespace std {
    using std::experimental::coroutine_handle;
    using std::experimental::noop_coroutine;
    using std::experimental::suspend_always;
    using std::experimental::suspend_never;
}
#endif  // __has_include(<coroutine>)

#include <atomic>
#include <iterator>
#include <memory>

#ifndef INCLUDED_CORO_MANUAL_LIFETIME_H
#define INCLUDED_CORO_MANUAL_LIFETIME_H

namespace coro {

    template <class T>
    struct manual_lifetime {
       public:
        manual_lifetime() noexcept {}

        ~manual_lifetime() noexcept {}

        template <class... Args>
        void construct(Args &&...args) {
            ::new (static_cast<void *>(std::addressof(value))) T(static_cast<Args &&>(args)...);
        }

        void destruct() noexcept { value.~T(); }

        T &get() & { return value; }

        const T &get() const & { return value; }

        T &&get() && { return (T &&) value; }

        const T &&get() const && { return (const T &&)value; }

       private:
        union {
            T value;
        };
    };

    template <class T>
    struct manual_lifetime<T &> {
        manual_lifetime() noexcept = default;

        void construct(T &value) noexcept { ptr = std::addressof(value); }

        void destruct() noexcept {}

        T &get() const noexcept { return *ptr; }

       private:
        T *ptr = nullptr;
    };

    template <class T>
    struct manual_lifetime<T &&> {
        manual_lifetime() noexcept = default;

        void construct(T &&value) noexcept { ptr = std::addressof(value); }

        void destruct() noexcept {}

        T &&get() const noexcept { return *ptr; }

       private:
        T *ptr = nullptr;
    };

    template <>
    struct manual_lifetime<void> {
        void construct() noexcept {}

        void destruct() noexcept {}

        void get() const noexcept {}
    };

#endif  // INCLUDED_CORO_MANUAL_LIFETIME_H

    template <class Ref, class Value = std::decay_t<Ref>>
    class shared_generator {
       public:
        class promise_type {
           public:
            promise_type() noexcept = default;

            ~promise_type() noexcept { clear_value(); }

            void clear_value() {
                if (hasValue_) {
                    hasValue_ = false;
                    ref_.destruct();
                }
            }

            shared_generator get_return_object() noexcept { return shared_generator{std::coroutine_handle<promise_type>::from_promise(*this)}; }

            auto initial_suspend() noexcept { return std::suspend_always{}; }

            auto final_suspend() noexcept { return std::suspend_always{}; }

            auto yield_value(Ref ref) noexcept(std::is_nothrow_move_constructible_v<Ref>) {
                ref_.construct(std::move(ref));
                hasValue_ = true;
                return std::suspend_always{};
            }

            void return_void() {}

            void unhandled_exception() { throw; }

            Ref get() { return ref_.get(); }

           private:
            friend class shared_generator;

            manual_lifetime<Ref> ref_;
            bool hasValue_ = false;
            std::atomic<int> refcount_{1};
        };

        using handle_t = std::coroutine_handle<promise_type>;

        // ViewableRange refines Semiregular refines DefaultConstructible
        explicit shared_generator() = default;

        shared_generator(shared_generator &&g) noexcept : coro_(std::exchange(g.coro_, {})) {}

        // ViewableRange refines Semiregular refines Copyable
        shared_generator(const shared_generator &g) noexcept : coro_(g.coro_) {
            if (coro_) {
                ++coro_.promise().refcount_;
            }
        }

        // ViewableRange refines Semiregular refines Copyable
        shared_generator &operator=(shared_generator g) noexcept { this->swap(g); }

        void swap(shared_generator &g) noexcept {
            using std::swap;
            swap(coro_, g.coro_);
        }

        friend void swap(shared_generator &g, shared_generator &h) noexcept { g.swap(h); }

        ~shared_generator() {
            if (coro_) {
                if (--coro_.promise().refcount_ == 0) {
                    coro_.destroy();
                }
            }
        }

        struct sentinel {};

        class iterator {
           public:
            using reference = Ref;
            using value_type = Value;
            using difference_type = std::ptrdiff_t;
            using pointer = std::add_pointer_t<Ref>;
            using iterator_category = std::input_iterator_tag;

            iterator() noexcept = default;

            explicit iterator(handle_t coro) noexcept : coro_(coro) {}

            reference operator*() const { return coro_.promise().get(); }

            iterator &operator++() {
                coro_.promise().clear_value();
                coro_.resume();
                return *this;
            }

            void operator++(int) {
                coro_.promise().clear_value();
                coro_.resume();
            }

            friend bool operator==(const iterator &it, sentinel) noexcept { return it.coro_.done(); }

            friend bool operator==(sentinel, const iterator &it) noexcept { return it.coro_.done(); }

            friend bool operator!=(const iterator &it, sentinel) noexcept { return !it.coro_.done(); }

            friend bool operator!=(sentinel, const iterator &it) noexcept { return !it.coro_.done(); }

           private:
            handle_t coro_;
        };

        iterator begin() {
            coro_.resume();
            return iterator{coro_};
        }

        sentinel end() { return {}; }

       private:
        explicit shared_generator(handle_t coro) noexcept : coro_(coro) {}

        handle_t coro_;
    };
}

#endif  // INCLUDED_CORO_SHARED_GENERATOR_H

//------------------
//general coroutine support
//------------------
#if defined(__clang__)
    //see: https://developercommunity.visualstudio.com/content/problem/502513/unable-to-use-clang-cl-coroutines-due-to-unresolve.html
    namespace std {
        namespace experimental {
            inline namespace coroutines_v1 {

                template <typename R, typename...>
                struct coroutine_traits {
                    using promise_type = typename R::promise_type;
                };

                template <typename Promise = void>
                struct coroutine_handle;

                template <> struct coroutine_handle<void> {
                    static coroutine_handle from_address(void* addr) noexcept {
                        coroutine_handle me;
                        me.ptr = addr;
                        return me;
                    }
                    void operator()() { resume(); }
                    void* address() const { return ptr; }
                    void resume() const { __builtin_coro_resume(ptr); }
                    void destroy() const { __builtin_coro_destroy(ptr); }
                    bool done() const { return __builtin_coro_done(ptr); }
                    coroutine_handle& operator=(decltype(nullptr)) {
                        ptr = nullptr;
                        return *this;
                    }
                    coroutine_handle(decltype(nullptr)) : ptr(nullptr) {}
                    coroutine_handle() : ptr(nullptr) {}
                    //  void reset() { ptr = nullptr; } // add to P0057?
                    explicit operator bool() const { return ptr; }

                protected:
                    void* ptr;
                };

                template <typename Promise>
                struct coroutine_handle : coroutine_handle<> {
                    using coroutine_handle<>::operator=;
                    using coroutine_handle<>::coroutine_handle;

                    static coroutine_handle from_address(void* addr) noexcept {
                        coroutine_handle me;
                        me.ptr = addr;
                        return me;
                    }

                    Promise& promise() const {
                        return *reinterpret_cast<Promise*>(
                            __builtin_coro_promise(ptr, alignof(Promise), false));
                    }
                    static coroutine_handle from_promise(Promise& promise) {
                        coroutine_handle p;
                        p.ptr = __builtin_coro_promise(&promise, alignof(Promise), true);
                        return p;
                    }
                };

                template <typename _PromiseT>
                bool operator==(coroutine_handle<_PromiseT> const& _Left,
                    coroutine_handle<_PromiseT> const& _Right) noexcept
                {
                    return _Left.address() == _Right.address();
                }

                template <typename _PromiseT>
                bool operator!=(coroutine_handle<_PromiseT> const& _Left,
                    coroutine_handle<_PromiseT> const& _Right) noexcept
                {
                    return !(_Left == _Right);
                }

                struct suspend_always {
                    bool await_ready() noexcept { return false; }
                    void await_suspend(coroutine_handle<>) noexcept {}
                    void await_resume() noexcept {}
                };
                struct suspend_never {
                    bool await_ready() noexcept { return true; }
                    void await_suspend(coroutine_handle<>) noexcept {}
                    void await_resume() noexcept {}
                };
            }
        }
    }
    #define NATIVE_COROUTINE_IMPL_NS std::experimental
#elif __has_include(<coroutine>)
    #include <coroutine>
    #define NATIVE_COROUTINE_IMPL_NS std
#elif __has_include(<experimental/coroutine>)
    #include <experimental/coroutine>
    #define NATIVE_COROUTINE_IMPL_NS std::experimental
#endif

//------------------
//Generator defintion
//-----------------

#if __has_include(<generator>)
    #include <generator>
    namespace CORE_NATIVE_NS
    {
        template<typename T>
        using YieldEnum = std::generator<T>;
    }
#elif __has_include(<experimental/generator>) && !defined(__clang__)
    #include <experimental/generator>
    namespace CORE_NATIVE_NS
    {
        template<typename T>
        using YieldEnum = std::experimental::generator<T>;
    }
#else
#include <cstddef>
#include <exception>
#include <functional>
#include <iterator>
#include <memory>
#include <type_traits>
#include <utility>

namespace CORE_NATIVE_NS
{
    template<typename T>
    class generator;

    namespace detail
    {
        template<typename T>
        class generator_promise
        {
        public:
            using value_type = std::remove_reference_t<T>;
            using reference_type = std::conditional_t<std::is_reference_v<T>, T, T&>;
            using pointer_type = value_type*;

            generator_promise() = default;

            generator<T> get_return_object() noexcept;

            constexpr NATIVE_COROUTINE_IMPL_NS::suspend_always initial_suspend() const noexcept
            { return {}; }
            constexpr NATIVE_COROUTINE_IMPL_NS::suspend_always final_suspend() const noexcept
            { return {}; }

            template<
                typename U = T,
                std::enable_if_t<!std::is_rvalue_reference<U>::value, int> = 0
            >
            NATIVE_COROUTINE_IMPL_NS::suspend_always yield_value(std::remove_reference_t<T>& value) noexcept
            {
                m_value = std::addressof(value);
                return {};
            }

            NATIVE_COROUTINE_IMPL_NS::suspend_always yield_value(std::remove_reference_t<T>&& value) noexcept
            {
                m_value = std::addressof(value);
                return {};
            }

            void unhandled_exception() noexcept
            {
                m_exception = std::current_exception();
            }

            void return_void() const noexcept
            {
            }

            reference_type value() const noexcept
            {
                return static_cast<reference_type>(*m_value);
            }

            // Don't allow any use of 'co_await' inside the generator coroutine.
            template<typename U>
            NATIVE_COROUTINE_IMPL_NS::suspend_never await_transform(U&& value) = delete;

            void rethrow_if_exception()
            {
                if (m_exception)
                {
                    std::rethrow_exception(m_exception);
                }
            }
        private:
            pointer_type m_value;
            std::exception_ptr m_exception;
        };

        struct generator_sentinel {};

        template<typename T>
        class generator_iterator
        {
            using coroutine_handle = NATIVE_COROUTINE_IMPL_NS::coroutine_handle<generator_promise<T>>;
        public:

            using iterator_category = std::input_iterator_tag;
            // What type should we use for counting elements of a potentially infinite sequence?
            using difference_type = std::ptrdiff_t;
            using value_type = typename generator_promise<T>::value_type;
            using reference = typename generator_promise<T>::reference_type;
            using pointer = typename generator_promise<T>::pointer_type;

            // Iterator needs to be default-constructible to satisfy the Range concept.
            generator_iterator() noexcept
                : m_coroutine(nullptr)
            {}

            explicit generator_iterator(coroutine_handle coroutine) noexcept
                : m_coroutine(coroutine)
            {}

            friend bool operator==(const generator_iterator& it, generator_sentinel) noexcept
            {
                return !it.m_coroutine || it.m_coroutine.done();
            }

            friend bool operator!=(const generator_iterator& it, generator_sentinel s) noexcept
            {
                return !(it == s);
            }

            friend bool operator==(generator_sentinel s, const generator_iterator& it) noexcept
            {
                return (it == s);
            }

            friend bool operator!=(generator_sentinel s, const generator_iterator& it) noexcept
            {
                return it != s;
            }

            generator_iterator& operator++()
            {
                m_coroutine.resume();
                if (m_coroutine.done())
                {
                    m_coroutine.promise().rethrow_if_exception();
                }

                return *this;
            }

            // Need to provide post-increment operator to implement the 'Range' concept.
            void operator++(int)
            {
                (void)operator++();
            }

            void next(){
                (void)operator++();
            }

            reference operator*() const noexcept
            {
                return m_coroutine.promise().value();
            }

            pointer operator->() const noexcept
            {
                return std::addressof(operator*());
            }

        private:
            coroutine_handle m_coroutine;
        };
    }

    template<typename T>
    class [[nodiscard]] generator
    {
    public:

        using promise_type = detail::generator_promise<T>;
        using iterator = detail::generator_iterator<T>;

        generator() noexcept
            : m_coroutine(nullptr)
        {}

        generator(generator&& other) noexcept
            : m_coroutine(other.m_coroutine)
        {
            other.m_coroutine = nullptr;
        }

        generator(const generator& other) = delete;

        ~generator()
        {
            if (m_coroutine)
            {
                m_coroutine.destroy();
            }
        }

        generator& operator=(generator other) noexcept
        {
            swap(other);
            return *this;
        }

        iterator begin()
        {
            if (m_coroutine)
            {
                m_coroutine.resume();
                if (m_coroutine.done())
                {
                    m_coroutine.promise().rethrow_if_exception();
                }
            }

            return iterator{ m_coroutine };
        }

        detail::generator_sentinel end() noexcept
        {
            return detail::generator_sentinel{};
        }

        void swap(generator& other) noexcept
        {
            std::swap(m_coroutine, other.m_coroutine);
        }

    private:

        friend class detail::generator_promise<T>;

        explicit generator(NATIVE_COROUTINE_IMPL_NS::coroutine_handle<promise_type> coroutine) noexcept
            : m_coroutine(coroutine)
        {}

        NATIVE_COROUTINE_IMPL_NS::coroutine_handle<promise_type> m_coroutine;

    };

    template<typename T>
    void swap(generator<T>& a, generator<T>& b)
    {
        a.swap(b);
    }

    namespace detail
    {
        template<typename T>
        generator<T> generator_promise<T>::get_return_object() noexcept
        {
            using coroutine_handle = NATIVE_COROUTINE_IMPL_NS::coroutine_handle<generator_promise<T>>;
            return generator<T>{ coroutine_handle::from_promise(*this) };
        }
    }

    template<typename FUNC, typename T>
    generator<std::invoke_result_t<FUNC&, typename generator<T>::iterator::reference>> fmap(FUNC func, generator<T> source)
    {
        for (auto&& value : source)
        {
            co_yield std::invoke(func, static_cast<decltype(value)>(value));
        }
    }

    template<typename T>
    using YieldEnum = CORE_NATIVE_NS::generator<T>;
}
#endif

 */