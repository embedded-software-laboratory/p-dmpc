#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iterator>

#if __has_include(<memory_resource>)
#include <memory_resource>
#else
#include <experimental/memory_resource>
#endif

class PointerPostfix {
	std::uint8_t *&ptr;
	std::ptrdiff_t const add;

   public:
	PointerPostfix(std::uint8_t *&ptr, std::ptrdiff_t const add) : ptr(ptr), add(add) {}
	operator std::uint8_t *() { return ptr; }
	~PointerPostfix() { ptr += add; }
};

namespace GraphBasedPlanning {
	template <typename T = double>
	class Node {
		T _g;
		T _h;
		Node const *const _parent;
		std::uint8_t const _k;
		unsigned int _children_size;
		Node **_children;

		static unsigned int _n_vehicles;
		static std::ptrdiff_t _trim_offset;
		static std::ptrdiff_t _x_offset;
		static std::ptrdiff_t _y_offset;
		static std::ptrdiff_t _yaw_offset;
		static std::ptrdiff_t _max;

	   public:
		void operator delete(void *ptr) { std::free(ptr); }

		static void init_static_variables(unsigned int const n_vehicles) {
			_n_vehicles = n_vehicles;

			std::ptrdiff_t const size_trim_vector = _n_vehicles * sizeof(std::uint8_t);
			std::ptrdiff_t const size_x_vector = _n_vehicles * sizeof(T);
			std::ptrdiff_t const size_y_vector = _n_vehicles * sizeof(T);
			std::ptrdiff_t const size_yaw_vector = _n_vehicles * sizeof(T);

			_trim_offset = sizeof(Node);
			_x_offset = _trim_offset + aligned_offset(size_trim_vector);
			_y_offset = _x_offset + aligned_offset(size_x_vector);
			_yaw_offset = _y_offset + aligned_offset(size_y_vector);
			_max = _yaw_offset + aligned_offset(size_yaw_vector);
		}

		template <typename TrimInputIterator, typename XInputIterator, typename YInputIterator, typename YawInputIterator>
		    requires std::input_iterator<TrimInputIterator> && std::is_same_v<typename std::iterator_traits<TrimInputIterator>::value_type, std::uint8_t> && std::input_iterator<XInputIterator> &&
		             std::is_same_v<typename std::iterator_traits<XInputIterator>::value_type, T> && std::input_iterator<YInputIterator> && std::is_same_v<typename std::iterator_traits<YInputIterator>::value_type, T> &&
		             std::input_iterator<YawInputIterator> && std::is_same_v<typename std::iterator_traits<YawInputIterator>::value_type, T>
		[[nodiscard]] static Node *create_root_node(TrimInputIterator trim_begin, XInputIterator x_begin, YInputIterator y_begin, YawInputIterator yaw_begin) {
			return new Node(std::forward<TrimInputIterator>(trim_begin), std::forward<XInputIterator>(x_begin), std::forward<YInputIterator>(y_begin), std::forward<YawInputIterator>(yaw_begin));
		}

		[[nodiscard]] static Node const *create_copy(Node const *const node, Node const *const parent) { return new Node(node->g(), node->h(), parent, node->k(), node->trim_ptr(), node->x_ptr(), node->y_ptr(), node->yaw_ptr()); }

		template <typename TrimInputIterator, typename XInputIterator, typename YInputIterator, typename YawInputIterator>
		    requires std::input_iterator<TrimInputIterator> && std::is_same_v<typename std::iterator_traits<TrimInputIterator>::value_type, std::uint8_t> && std::input_iterator<XInputIterator> &&
		             std::is_same_v<typename std::iterator_traits<XInputIterator>::value_type, T> && std::input_iterator<YInputIterator> && std::is_same_v<typename std::iterator_traits<YInputIterator>::value_type, T> &&
		             std::input_iterator<YawInputIterator> && std::is_same_v<typename std::iterator_traits<YawInputIterator>::value_type, T>
		[[nodiscard]] static Node *create_root_node_with_tracking_ptr(TrimInputIterator trim_begin, XInputIterator x_begin, YInputIterator y_begin, YawInputIterator yaw_begin, std::uint8_t *&tracking_ptr) {
			return new (PointerPostfix(tracking_ptr, _max)) Node(std::forward<TrimInputIterator>(trim_begin), std::forward<XInputIterator>(x_begin), std::forward<YInputIterator>(y_begin), std::forward<YawInputIterator>(yaw_begin));
		}

		template <typename TrimInputIterator, typename XInputIterator, typename YInputIterator, typename YawInputIterator>
		    requires std::input_iterator<TrimInputIterator> && std::is_same_v<typename std::iterator_traits<TrimInputIterator>::value_type, std::uint8_t> && std::input_iterator<XInputIterator> &&
		             std::is_same_v<typename std::iterator_traits<XInputIterator>::value_type, T> && std::input_iterator<YInputIterator> && std::is_same_v<typename std::iterator_traits<YInputIterator>::value_type, T> &&
		             std::input_iterator<YawInputIterator> && std::is_same_v<typename std::iterator_traits<YawInputIterator>::value_type, T>
		[[nodiscard]] static Node *create_root_node_with_polymorphic_allocator(TrimInputIterator trim_begin, XInputIterator x_begin, YInputIterator y_begin, YawInputIterator yaw_begin, std::pmr::polymorphic_allocator<> &allocator) {
			auto ptr = allocator.allocate_bytes(_max, alignof(Node));

			return new (ptr) Node(std::forward<TrimInputIterator>(trim_begin), std::forward<XInputIterator>(x_begin), std::forward<YInputIterator>(y_begin), std::forward<YawInputIterator>(yaw_begin));
		}

		[[nodiscard]] static Node *create_node(T const g_, /*T const h_,*/ Node const *const parent_, std::uint8_t const k_) {
			return new Node(std::forward<T const>(g_), /*std::forward<T const>(h_),*/ std::forward<decltype(parent_)>(parent_), std::forward<decltype(k_)>(k_));
		}

		[[nodiscard]] static Node *create_node_with_tracking_ptr(T const g_, /*T const h_,*/ Node const *const parent_, std::uint8_t const k_, std::uint8_t *&tracking_ptr) {
			return new (PointerPostfix(tracking_ptr, _max)) Node(std::forward<T const>(g_), /*std::forward<T const>(h_),*/ std::forward<decltype(parent_)>(parent_), std::forward<decltype(k_)>(k_));
		}

		[[nodiscard]] static Node *create_node_with_polymorphic_allocator(T const g_, /*T const h_,*/ Node const *const parent_, std::uint8_t const k_, std::pmr::polymorphic_allocator<> &allocator) {
			auto *ptr = allocator.allocate_bytes(_max, alignof(Node));
			return new (ptr) Node(std::forward<T const>(g_), /*std::forward<T const>(h_),*/ std::forward<decltype(parent_)>(parent_), std::forward<decltype(k_)>(k_));
		}

		[[nodiscard]] static std::ptrdiff_t max() { return _max; }

		[[nodiscard]] T &g() { return _g; }
		[[nodiscard]] T g() const { return _g; }

		[[nodiscard]] T &h() { return _h; }
		[[nodiscard]] T h() const { return _h; }

		[[nodiscard]] Node const *parent() const { return _parent; }

		[[nodiscard]] std::uint8_t k() const { return _k; }

		[[nodiscard]] unsigned int children_size() const { return _children_size; }

		[[nodiscard]] Node *&children(std::size_t const index) { return _children[index]; }
		[[nodiscard]] Node *&children(std::size_t const index) const { return _children[index]; }

		[[nodiscard]] std::uint8_t &trim(unsigned int const vehicle) { return *(std::uint8_t *)((std::byte *)this + _trim_offset + sizeof(std::uint8_t) * vehicle); }
		[[nodiscard]] std::uint8_t trim(unsigned int const vehicle) const { return *(std::uint8_t *)((std::byte *)this + _trim_offset + sizeof(std::uint8_t) * vehicle); }
		[[nodiscard]] std::uint8_t const *trim_ptr() const { return (std::uint8_t *)((std::byte *)this + _trim_offset); }
		[[nodiscard]] std::uint8_t *trim_ptr() { return (std::uint8_t *)((std::byte *)this + _trim_offset); }

		[[nodiscard]] T &x(unsigned int const vehicle) { return *(T *)((std::byte *)this + _x_offset + sizeof(T) * vehicle); }
		[[nodiscard]] T x(unsigned int const vehicle) const { return *(T *)((std::byte *)this + _x_offset + sizeof(T) * vehicle); }
		[[nodiscard]] T const *x_ptr() const { return (T *)((std::byte *)this + _x_offset); }
		[[nodiscard]] T *x_ptr() { return (T *)((std::byte *)this + _x_offset); }

		[[nodiscard]] T &y(unsigned int const vehicle) { return *(T *)((std::byte *)this + _y_offset + sizeof(T) * vehicle); }
		[[nodiscard]] T y(unsigned int const vehicle) const { return *(T *)((std::byte *)this + _y_offset + sizeof(T) * vehicle); }
		[[nodiscard]] T const *y_ptr() const { return (T *)((std::byte *)this + _y_offset); }
		[[nodiscard]] T *y_ptr() { return (T *)((std::byte *)this + _y_offset); }

		[[nodiscard]] T &yaw(unsigned int const vehicle) { return *(T *)((std::byte *)this + _yaw_offset + sizeof(T) * vehicle); }
		[[nodiscard]] T yaw(unsigned int const vehicle) const { return *(T *)((std::byte *)this + _yaw_offset + sizeof(T) * vehicle); }
		[[nodiscard]] T const *yaw_ptr() const { return (T *)((std::byte *)this + _yaw_offset); }
		[[nodiscard]] T *yaw_ptr() { return (T *)((std::byte *)this + _yaw_offset); }

		void create_children(std::size_t const size) {
			_children_size = size;
			_children = new Node *[size] { nullptr };
		}

		void delete_and_delete_from_parent() const {
			for (std::size_t i = 0; i < parent()->children_size(); ++i) {
				if (parent()->children(i) == this) {
					parent()->children(i) = nullptr;
				}
			}

			delete this;
		}

		void create_children_with_polymorphic_allocator(std::size_t const size, std::pmr::polymorphic_allocator<> &allocator) {
			_children_size = size;
			_children = allocator.allocate_object<Node *>(size);
		}

		~Node() {
			for (std::size_t i = 0; i < _children_size; ++i) {
				delete _children[i];
			}

			delete[] _children;
		}

	   private:
		void *operator new([[maybe_unused]] std::size_t size) { return std::malloc(_max); }

		void *operator new([[maybe_unused]] std::size_t size, void *ptr) { return ptr; }

		template <typename TrimInputIterator, typename XInputIterator, typename YInputIterator, typename YawInputIterator>
		Node(TrimInputIterator trim_begin, XInputIterator x_begin, YInputIterator y_begin, YawInputIterator yaw_begin) : _g(0U), _h(0U), _parent(nullptr), _k(0U), _children_size(0U), _children(nullptr) {
			std::copy(trim_begin, trim_begin + _n_vehicles, trim_ptr());
			std::copy(x_begin, x_begin + _n_vehicles, x_ptr());
			std::copy(y_begin, y_begin + _n_vehicles, y_ptr());
			std::copy(yaw_begin, yaw_begin + _n_vehicles, yaw_ptr());
		}

		Node(T const g_, /*T const h_,*/ Node const *const parent_, std::uint8_t const k_) : _g(g_), _h(0.0/*h_*/), _parent(parent_), _k(k_), _children_size(0U), _children(nullptr) {}

		// only for copying
		template <typename TrimInputIterator, typename XInputIterator, typename YInputIterator, typename YawInputIterator>
		Node(T const g_, T const h_, Node const *const parent_, std::uint8_t const k_, TrimInputIterator trim_begin, XInputIterator x_begin, YInputIterator y_begin, YawInputIterator yaw_begin)
		    : _g(g_), _h(h_), _parent(parent_), _k(k_), _children_size(0U), _children(nullptr) {
			std::copy(trim_begin, trim_begin + _n_vehicles, trim_ptr());
			std::copy(x_begin, x_begin + _n_vehicles, x_ptr());
			std::copy(y_begin, y_begin + _n_vehicles, y_ptr());
			std::copy(yaw_begin, yaw_begin + _n_vehicles, yaw_ptr());
		}

		static constexpr std::ptrdiff_t aligned_offset(std::ptrdiff_t const size_vector) {
			if (size_vector % alignof(Node)) {
				return size_vector + alignof(Node) - (size_vector % alignof(Node));
			} else {
				return size_vector;
			}
		}

	   public:
		friend bool operator<(Node const &lhs, Node const &rhs) { return lhs.g() + lhs.h() < rhs.g() + rhs.h(); }
		friend bool operator>(Node const &lhs, Node const &rhs) { return lhs.g() + lhs.h() > rhs.g() + rhs.h(); }

		friend bool operator==(Node const &lhs, Node const &rhs) {
			for (unsigned int i = 0; i < _n_vehicles; ++i) {
				if (lhs.x(i) != rhs.x(i)) return false;
				if (lhs.y(i) != rhs.y(i)) return false;
				if (lhs.trim(i) != rhs.trim(i)) return false;
			}

			return true;
		}

		struct priority_queue_comparison {
			bool operator()(Node const *const lhs, Node const *const rhs) const { return *lhs > *rhs; }
		};

		struct priority_queue_comparison_less {
			bool operator()(Node const *const lhs, Node const *const rhs) const { return *lhs < *rhs; }
		};
	};

	template <typename T>
	unsigned int Node<T>::_n_vehicles = 0U;
	template <typename T>
	std::ptrdiff_t Node<T>::_trim_offset = 0;
	template <typename T>
	std::ptrdiff_t Node<T>::_x_offset = 0;
	template <typename T>
	std::ptrdiff_t Node<T>::_y_offset = 0;
	template <typename T>
	std::ptrdiff_t Node<T>::_yaw_offset = 0;
	template <typename T>
	std::ptrdiff_t Node<T>::_max = sizeof(Node<T>);

	/*template <typename T>
	class Node {
	   protected:
	    T _g;
	    T _h;
	    Node *const _parent;

	    std::uint8_t const _k;

	    virtual std::ptrdiff_t trim_offset() const = 0;
	    virtual std::ptrdiff_t x_offset() const = 0;
	    virtual std::ptrdiff_t y_offset() const = 0;
	    virtual std::ptrdiff_t yaw_offset() const = 0;
	    virtual std::ptrdiff_t max() const = 0;

	    static constexpr std::ptrdiff_t aligned_offset(std::ptrdiff_t const size_vector) {
	        if (size_vector % alignof(Node)) {
	            return size_vector + alignof(Node) - (size_vector % alignof(Node));
	        } else {
	            return size_vector;
	        }
	    }

	   public:
	    [[nodiscard]] std::uint8_t k() const { return _k; }

	    [[nodiscard]] T &g() { return _g; }
	    [[nodiscard]] T g() const { return _g; }

	    [[nodiscard]] T &h() { return _h; }
	    [[nodiscard]] T h() const { return _h; }

	    [[nodiscard]] Node *parent() const { return _parent; }

	    [[nodiscard]] std::uint8_t &trim(unsigned int const vehicle) { return *(std::uint8_t *)((std::uint8_t *)this + trim_offset() + sizeof(std::uint8_t) * vehicle); }
	    [[nodiscard]] std::uint8_t trim(unsigned int const vehicle) const { return *(std::uint8_t *)((std::uint8_t *)this + trim_offset() + sizeof(std::uint8_t) * vehicle); }

	    [[nodiscard]] T &x(unsigned int const vehicle) { return *(T *)((std::uint8_t *)this + x_offset() + sizeof(T) * vehicle); }
	    [[nodiscard]] T x(unsigned int const vehicle) const { return *(T *)((std::uint8_t *)this + x_offset() + sizeof(T) * vehicle); }

	    [[nodiscard]] T &y(unsigned int const vehicle) { return *(T *)((std::uint8_t *)this + y_offset() + sizeof(T) * vehicle); }
	    [[nodiscard]] T y(unsigned int const vehicle) const { return *(T *)((std::uint8_t *)this + y_offset() + sizeof(T) * vehicle); }

	    [[nodiscard]] T &yaw(unsigned int const vehicle) { return *(T *)((std::uint8_t *)this + yaw_offset() + sizeof(T) * vehicle); }
	    [[nodiscard]] T yaw(unsigned int const vehicle) const { return *(T *)((std::uint8_t *)this + yaw_offset() + sizeof(T) * vehicle); }

	    virtual ~Node() {}

	   protected:
	    Node(std::uint8_t const k_, T const g_, T const h_, Node *const parent_) : _g(g_), _h(h_), _parent(parent_), _k(k_) {}

	   public:
	    friend bool operator<(Node const &lhs, Node const &rhs) { return lhs.g() + lhs.h() < rhs.g() + rhs.h(); }
	    friend bool operator>(Node const &lhs, Node const &rhs) { return lhs.g() + lhs.h() > rhs.g() + rhs.h(); }

	    struct priority_queue_comparison {
	        bool operator()(Node const *const lhs, Node const *const rhs) const { return *lhs > *rhs; }
	    };
	};

	template <typename T>
	class NodeMonteCarlo : public Node<T> {
	    static std::ptrdiff_t _trim_offset;
	    static std::ptrdiff_t _x_offset;
	    static std::ptrdiff_t _y_offset;
	    static std::ptrdiff_t _yaw_offset;
	    static std::ptrdiff_t _max;

	    std::ptrdiff_t trim_offset() const final { return _trim_offset; }
	    std::ptrdiff_t x_offset() const final { return _x_offset; }
	    std::ptrdiff_t y_offset() const final { return _y_offset; }
	    std::ptrdiff_t yaw_offset() const final { return _yaw_offset; }
	    std::ptrdiff_t max() const final { return _max; }

	    std::uint8_t dummy;

	   public:
	    template <typename TrimInputIterator, typename XInputIterator, typename YInputIterator, typename YawInputIterator>
	        requires std::input_iterator<TrimInputIterator> && std::is_same_v<typename std::iterator_traits<TrimInputIterator>::value_type, std::uint8_t> && std::input_iterator<XInputIterator> &&
	                 std::is_same_v<typename std::iterator_traits<XInputIterator>::value_type, T> && std::input_iterator<YInputIterator> && std::is_same_v<typename std::iterator_traits<YInputIterator>::value_type, T> &&
	                 std::input_iterator<YawInputIterator> && std::is_same_v<typename std::iterator_traits<YawInputIterator>::value_type, T>
	    [[nodiscard]] static NodeMonteCarlo *create_root_node(std::size_t const n_vehicles, TrimInputIterator trim_begin, XInputIterator x_begin, YInputIterator y_begin, YawInputIterator yaw_begin) {
	        // init static variables:
	        std::ptrdiff_t const size_trim_vector = n_vehicles * sizeof(std::uint8_t);
	        std::ptrdiff_t const size_x_vector = n_vehicles * sizeof(T);
	        std::ptrdiff_t const size_y_vector = n_vehicles * sizeof(T);
	        std::ptrdiff_t const size_yaw_vector = n_vehicles * sizeof(T);

	        _trim_offset = offsetof(NodeMonteCarlo, dummy);  // sizeof(Node<T>);

	        if (_trim_offset % alignof(NodeMonteCarlo)) {
	            if (size_trim_vector + 1U % alignof(NodeMonteCarlo)) {
	                _x_offset = _trim_offset + size_trim_vector + alignof(NodeMonteCarlo) - ((size_trim_vector + 1U) % alignof(NodeMonteCarlo));
	            } else {
	                _x_offset = _trim_offset + size_trim_vector;
	            }
	        } else {
	            _x_offset = _trim_offset + NodeMonteCarlo::aligned_offset(size_trim_vector);
	        }

	        _y_offset = _x_offset + NodeMonteCarlo::aligned_offset(size_x_vector);
	        _yaw_offset = _y_offset + NodeMonteCarlo::aligned_offset(size_y_vector);
	        _max = _yaw_offset + NodeMonteCarlo::aligned_offset(size_yaw_vector);

	        return new NodeMonteCarlo(
	            std::forward<std::size_t const>(n_vehicles), std::forward<TrimInputIterator>(trim_begin), std::forward<XInputIterator>(x_begin), std::forward<YInputIterator>(y_begin), std::forward<YawInputIterator>(yaw_begin));
	    }

	    static NodeMonteCarlo *create_node(std::uint8_t const k_, T const g_, T const h_, NodeMonteCarlo *const parent_) {
	        return new NodeMonteCarlo(std::forward<std::uint8_t const>(k_), std::forward<T const>(g_), std::forward<T const>(h_), std::forward<NodeMonteCarlo *const>(parent_));
	    }

	    void operator delete(void *ptr) { std::free(ptr); }

	   private:
	    NodeMonteCarlo(std::uint8_t const k_, T const g_, T const h_, NodeMonteCarlo *const parent_) : Node<T>(k_, g_, h_, parent_) {}

	    template <typename TrimInputIterator, typename XInputIterator, typename YInputIterator, typename YawInputIterator>
	    NodeMonteCarlo(std::size_t const n_vehicles, TrimInputIterator trim_begin, XInputIterator x_begin, YInputIterator y_begin, YawInputIterator yaw_begin) : Node<T>(0U, 0U, 0U, nullptr) {
	        std::copy(trim_begin, trim_begin + n_vehicles, &NodeMonteCarlo::trim(0U));
	        std::copy(x_begin, x_begin + n_vehicles, &NodeMonteCarlo::x(0U));
	        std::copy(y_begin, y_begin + n_vehicles, &NodeMonteCarlo::y(0U));
	        std::copy(yaw_begin, yaw_begin + n_vehicles, &NodeMonteCarlo::yaw(0U));
	    }

	    void *operator new(std::size_t size) {
	        //std::cout << size << " vs. " << _max << std::endl;
	        return std::malloc(_max);
	    }
	};

	template <typename T>
	std::ptrdiff_t NodeMonteCarlo<T>::_trim_offset = 0;
	template <typename T>
	std::ptrdiff_t NodeMonteCarlo<T>::_x_offset = 0;
	template <typename T>
	std::ptrdiff_t NodeMonteCarlo<T>::_y_offset = 0;
	template <typename T>
	std::ptrdiff_t NodeMonteCarlo<T>::_yaw_offset = 0;
	template <typename T>
	std::ptrdiff_t NodeMonteCarlo<T>::_max = sizeof(NodeMonteCarlo<T>);

	template <typename T>
	class NodeOptimal : public Node<T> {
	    static std::ptrdiff_t _trim_offset;
	    static std::ptrdiff_t _x_offset;
	    static std::ptrdiff_t _y_offset;
	    static std::ptrdiff_t _yaw_offset;
	    static std::ptrdiff_t _max;

	    std::ptrdiff_t trim_offset() const final { return _trim_offset; }
	    std::ptrdiff_t x_offset() const final { return _x_offset; }
	    std::ptrdiff_t y_offset() const final { return _y_offset; }
	    std::ptrdiff_t yaw_offset() const final { return _yaw_offset; }
	    std::ptrdiff_t max() const final { return _max; }

	    unsigned int _children_size;
	    NodeOptimal **_children;

	   public:
	    void create_children(std::size_t const size) {
	        _children_size = size;
	        _children = new NodeOptimal *[size] { nullptr };
	    }

	    [[nodiscard]] unsigned int children_size() const { return _children_size; }

	    [[nodiscard]] NodeOptimal *&children(std::size_t const index) { return _children[index]; }
	    [[nodiscard]] NodeOptimal *children(std::size_t const index) const { return _children[index]; }

	    template <typename TrimInputIterator, typename XInputIterator, typename YInputIterator, typename YawInputIterator>
	        requires std::input_iterator<TrimInputIterator> && std::is_same_v<typename std::iterator_traits<TrimInputIterator>::value_type, std::uint8_t> && std::input_iterator<XInputIterator> &&
	                 std::is_same_v<typename std::iterator_traits<XInputIterator>::value_type, T> && std::input_iterator<YInputIterator> && std::is_same_v<typename std::iterator_traits<YInputIterator>::value_type, T> &&
	                 std::input_iterator<YawInputIterator> && std::is_same_v<typename std::iterator_traits<YawInputIterator>::value_type, T>
	    [[nodiscard]] static NodeOptimal *create_root_node(std::size_t const n_vehicles, TrimInputIterator trim_begin, XInputIterator x_begin, YInputIterator y_begin, YawInputIterator yaw_begin) {
	        // init static variables:
	        std::ptrdiff_t const size_trim_vector = n_vehicles * sizeof(std::uint8_t);
	        std::ptrdiff_t const size_x_vector = n_vehicles * sizeof(T);
	        std::ptrdiff_t const size_y_vector = n_vehicles * sizeof(T);
	        std::ptrdiff_t const size_yaw_vector = n_vehicles * sizeof(T);

	        _trim_offset = sizeof(NodeOptimal);
	        _x_offset = _trim_offset + NodeOptimal::aligned_offset(size_trim_vector);
	        _y_offset = _x_offset + NodeOptimal::aligned_offset(size_x_vector);
	        _yaw_offset = _y_offset + NodeOptimal::aligned_offset(size_y_vector);
	        _max = _yaw_offset + NodeOptimal::aligned_offset(size_yaw_vector);

	        return new NodeOptimal(
	            std::forward<std::size_t const>(n_vehicles), std::forward<TrimInputIterator>(trim_begin), std::forward<XInputIterator>(x_begin), std::forward<YInputIterator>(y_begin), std::forward<YawInputIterator>(yaw_begin));
	    }

	    static NodeOptimal *create_node(std::uint8_t const k_, T const g_, T const h_, NodeOptimal *const parent_) {
	        return new NodeOptimal(std::forward<std::uint8_t const>(k_), std::forward<T const>(g_), std::forward<T const>(h_), std::forward<NodeOptimal *const>(parent_));
	    }

	    ~NodeOptimal() {
	        if (children_size()) {
	            for (std::size_t i = 0; i < children_size(); ++i) {
	                delete children(i);
	            }

	            delete[] _children;
	        }
	    }

	    void operator delete(void *ptr) { std::free(ptr); }

	   private:
	    NodeOptimal(std::uint8_t const k_, T const g_, T const h_, NodeOptimal *const parent_) : Node<T>(k_, g_, h_, parent_), _children_size(0U) {}

	    template <typename TrimInputIterator, typename XInputIterator, typename YInputIterator, typename YawInputIterator>
	    NodeOptimal(std::size_t const n_vehicles, TrimInputIterator trim_begin, XInputIterator x_begin, YInputIterator y_begin, YawInputIterator yaw_begin) : Node<T>(0U, 0U, 0U, nullptr), _children_size(0U) {
	        std::copy(trim_begin, trim_begin + n_vehicles, &NodeOptimal::trim(0U));
	        std::copy(x_begin, x_begin + n_vehicles, &NodeOptimal::x(0U));
	        std::copy(y_begin, y_begin + n_vehicles, &NodeOptimal::y(0U));
	        std::copy(yaw_begin, yaw_begin + n_vehicles, &NodeOptimal::yaw(0U));
	    }

	    void *operator new(std::size_t size) {
	        //std::cout << size << " vs. " << _max << std::endl;
	        return std::malloc(_max);
	    }
	};

	template <typename T>
	std::ptrdiff_t NodeOptimal<T>::_trim_offset = 0;
	template <typename T>
	std::ptrdiff_t NodeOptimal<T>::_x_offset = 0;
	template <typename T>
	std::ptrdiff_t NodeOptimal<T>::_y_offset = 0;
	template <typename T>
	std::ptrdiff_t NodeOptimal<T>::_yaw_offset = 0;
	template <typename T>
	std::ptrdiff_t NodeOptimal<T>::_max = sizeof(NodeOptimal<T>); */
}  // namespace GraphBasedPlanning