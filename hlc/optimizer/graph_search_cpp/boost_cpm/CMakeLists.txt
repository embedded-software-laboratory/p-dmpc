set(Boost_Version 1.82.0)

CPMAddPackage(
        NAME boost_geometry_lib
        VERSION ${Boost_Version}
        GITHUB_REPOSITORY "boostorg/geometry"
        GIT_TAG "boost-${Boost_Version}"
        DOWNLOAD_ONLY True
)
CPMAddPackage(
        NAME boost_range_lib
        VERSION ${Boost_Version}
        GITHUB_REPOSITORY "boostorg/range"
        GIT_TAG "boost-${Boost_Version}"
        DOWNLOAD_ONLY True
)
CPMAddPackage(
        NAME boost_config_lib
        VERSION ${Boost_Version}
        GITHUB_REPOSITORY "boostorg/config"
        GIT_TAG "boost-${Boost_Version}"
        DOWNLOAD_ONLY True
)
CPMAddPackage(
        NAME boost_preprocessor_lib
        VERSION ${Boost_Version}
        GITHUB_REPOSITORY "boostorg/preprocessor"
        GIT_TAG "boost-${Boost_Version}"
        DOWNLOAD_ONLY True
)
CPMAddPackage(
        NAME boost_mpl_lib
        VERSION ${Boost_Version}
        GITHUB_REPOSITORY "boostorg/mpl"
        GIT_TAG "boost-${Boost_Version}"
        DOWNLOAD_ONLY True
)
CPMAddPackage(
        NAME boost_type_traits_lib
        VERSION ${Boost_Version}
        GITHUB_REPOSITORY "boostorg/type_traits"
        GIT_TAG "boost-${Boost_Version}"
        DOWNLOAD_ONLY True
)
CPMAddPackage(
        NAME boost_iterator_lib
        VERSION ${Boost_Version}
        GITHUB_REPOSITORY "boostorg/iterator"
        GIT_TAG "boost-${Boost_Version}"
        DOWNLOAD_ONLY True
)
CPMAddPackage(
        NAME boost_static_assert_lib
        VERSION ${Boost_Version}
        GITHUB_REPOSITORY "boostorg/static_assert"
        GIT_TAG "boost-${Boost_Version}"
        DOWNLOAD_ONLY True
)
CPMAddPackage(
        NAME boost_concept_check_lib
        VERSION ${Boost_Version}
        GITHUB_REPOSITORY "boostorg/concept_check"
        GIT_TAG "boost-${Boost_Version}"
        DOWNLOAD_ONLY True
)
CPMAddPackage(
        NAME boost_variant_lib
        VERSION ${Boost_Version}
        GITHUB_REPOSITORY "boostorg/variant"
        GIT_TAG "boost-${Boost_Version}"
        DOWNLOAD_ONLY True
)
CPMAddPackage(
        NAME boost_move_lib
        VERSION ${Boost_Version}
        GITHUB_REPOSITORY "boostorg/move"
        GIT_TAG "boost-${Boost_Version}"
        DOWNLOAD_ONLY True
)
CPMAddPackage(
        NAME boost_core_lib
        VERSION ${Boost_Version}
        GITHUB_REPOSITORY "boostorg/core"
        GIT_TAG "boost-${Boost_Version}"
        DOWNLOAD_ONLY True
)
CPMAddPackage(
        NAME boost_conversion_lib
        VERSION ${Boost_Version}
        GITHUB_REPOSITORY "boostorg/numeric_conversion"
        GIT_TAG "boost-${Boost_Version}"
        DOWNLOAD_ONLY True
)
CPMAddPackage(
        NAME boost_throw_exception_lib
        VERSION ${Boost_Version}
        GITHUB_REPOSITORY "boostorg/throw_exception"
        GIT_TAG "boost-${Boost_Version}"
        DOWNLOAD_ONLY True
)
CPMAddPackage(
        NAME boost_assert_lib
        VERSION ${Boost_Version}
        GITHUB_REPOSITORY "boostorg/assert"
        GIT_TAG "boost-${Boost_Version}"
        DOWNLOAD_ONLY True
)
CPMAddPackage(
        NAME boost_rational_lib
        VERSION ${Boost_Version}
        GITHUB_REPOSITORY "boostorg/rational"
        GIT_TAG "boost-${Boost_Version}"
        DOWNLOAD_ONLY True
)
CPMAddPackage(
        NAME boost_utility_lib
        VERSION ${Boost_Version}
        GITHUB_REPOSITORY "boostorg/utility"
        GIT_TAG "boost-${Boost_Version}"
        DOWNLOAD_ONLY True
)
CPMAddPackage(
        NAME boost_integer_lib
        VERSION ${Boost_Version}
        GITHUB_REPOSITORY "boostorg/integer"
        GIT_TAG "boost-${Boost_Version}"
        DOWNLOAD_ONLY True
)
CPMAddPackage(
        NAME boost_multiprecision_lib
        VERSION ${Boost_Version}
        GITHUB_REPOSITORY "boostorg/multiprecision"
        GIT_TAG "boost-${Boost_Version}"
        DOWNLOAD_ONLY True
)
CPMAddPackage(
        NAME boost_math_lib
        VERSION ${Boost_Version}
        GITHUB_REPOSITORY "boostorg/math"
        GIT_TAG "boost-${Boost_Version}"
        DOWNLOAD_ONLY True
        #OPTIONS "BOOST_MATH_STANDALONE 1"
)
CPMAddPackage(
        NAME boost_detail_lib
        VERSION ${Boost_Version}
        GITHUB_REPOSITORY "boostorg/detail"
        GIT_TAG "boost-${Boost_Version}"
        DOWNLOAD_ONLY True
)
CPMAddPackage(
        NAME boost_type_index_lib
        VERSION ${Boost_Version}
        GITHUB_REPOSITORY "boostorg/type_index"
        GIT_TAG "boost-${Boost_Version}"
        DOWNLOAD_ONLY True
)
CPMAddPackage(
        NAME boost_type_index_lib
        VERSION ${Boost_Version}
        GITHUB_REPOSITORY "boostorg/type_index"
        GIT_TAG "boost-${Boost_Version}"
        DOWNLOAD_ONLY True
)
CPMAddPackage(
        NAME boost_container_hash_lib
        VERSION ${Boost_Version}
        GITHUB_REPOSITORY "boostorg/container_hash"
        GIT_TAG "boost-${Boost_Version}"
        DOWNLOAD_ONLY True
)
CPMAddPackage(
        NAME boost_array_lib
        VERSION ${Boost_Version}
        GITHUB_REPOSITORY "boostorg/array"
        GIT_TAG "boost-${Boost_Version}"
        DOWNLOAD_ONLY True
)
CPMAddPackage(
        NAME boost_function_types_lib
        VERSION ${Boost_Version}
        GITHUB_REPOSITORY "boostorg/function_types"
        GIT_TAG "boost-${Boost_Version}"
        DOWNLOAD_ONLY True
)
CPMAddPackage(
        NAME boost_algorithm_lib
        VERSION ${Boost_Version}
        GITHUB_REPOSITORY "boostorg/algorithm"
        GIT_TAG "boost-${Boost_Version}"
        DOWNLOAD_ONLY True
)
CPMAddPackage(
        NAME boost_tuple_lib
        VERSION ${Boost_Version}
        GITHUB_REPOSITORY "boostorg/tuple"
        GIT_TAG "boost-${Boost_Version}"
        DOWNLOAD_ONLY True
)
CPMAddPackage(
        NAME boost_qvm_lib
        VERSION ${Boost_Version}
        GITHUB_REPOSITORY "boostorg/qvm"
        GIT_TAG "boost-${Boost_Version}"
        DOWNLOAD_ONLY True
)
CPMAddPackage(
        NAME boost_container_lib
        VERSION ${Boost_Version}
        GITHUB_REPOSITORY "boostorg/container"
        GIT_TAG "boost-${Boost_Version}"
        DOWNLOAD_ONLY True
)
CPMAddPackage(
        NAME boost_intrusive_lib
        VERSION ${Boost_Version}
        GITHUB_REPOSITORY "boostorg/intrusive"
        GIT_TAG "boost-${Boost_Version}"
        DOWNLOAD_ONLY True
)
CPMAddPackage(
        NAME boost_smart_ptr_lib
        VERSION ${Boost_Version}
        GITHUB_REPOSITORY "boostorg/smart_ptr"
        GIT_TAG "boost-${Boost_Version}"
        DOWNLOAD_ONLY True
)
CPMAddPackage(
        NAME boost_lexical_cast_lib
        VERSION ${Boost_Version}
        GITHUB_REPOSITORY "boostorg/lexical_cast"
        GIT_TAG "boost-${Boost_Version}"
        DOWNLOAD_ONLY True
)
CPMAddPackage(
        NAME boost_tokenizer_lib
        VERSION ${Boost_Version}
        GITHUB_REPOSITORY "boostorg/tokenizer"
        GIT_TAG "boost-${Boost_Version}"
        DOWNLOAD_ONLY True
)
CPMAddPackage(
        NAME boost_function_lib
        VERSION ${Boost_Version}
        GITHUB_REPOSITORY "boostorg/function"
        GIT_TAG "boost-${Boost_Version}"
        DOWNLOAD_ONLY True
)
CPMAddPackage(
        NAME boost_bind_lib
        VERSION ${Boost_Version}
        GITHUB_REPOSITORY "boostorg/bind"
        GIT_TAG "boost-${Boost_Version}"
        DOWNLOAD_ONLY True
)

CPMAddPackage(
        NAME boost_parameter_lib
        VERSION ${Boost_Version}
        GITHUB_REPOSITORY "boostorg/parameter"
        GIT_TAG "boost-${Boost_Version}"
        DOWNLOAD_ONLY True
)

CPMAddPackage(
        NAME boost_heap_lib
        VERSION ${Boost_Version}
        GITHUB_REPOSITORY "boostorg/heap"
        GIT_TAG "boost-${Boost_Version}"
        DOWNLOAD_ONLY True
)

CPMAddPackage(
        NAME boost_mp11_lib
        VERSION ${Boost_Version}
        GITHUB_REPOSITORY "boostorg/mp11"
        GIT_TAG "boost-${Boost_Version}"
        DOWNLOAD_ONLY True
)

#include_directories(${boost_geometry_lib_SOURCE_DIR}/include)
#include_directories(${boost_range_lib_SOURCE_DIR}/include)
#include_directories(${boost_config_lib_SOURCE_DIR}/include)
#include_directories(${boost_preprocessor_lib_SOURCE_DIR}/include)
#include_directories(${boost_mpl_lib_SOURCE_DIR}/include)
#include_directories(${boost_type_traits_lib_SOURCE_DIR}/include)
#include_directories(${boost_iterator_lib_SOURCE_DIR}/include)
#include_directories(${boost_static_assert_lib_SOURCE_DIR}/include)
#include_directories(${boost_concept_check_lib_SOURCE_DIR}/include)
#include_directories(${boost_variant_lib_SOURCE_DIR}/include)
#include_directories(${boost_move_lib_SOURCE_DIR}/include)
#include_directories(${boost_core_lib_SOURCE_DIR}/include)
#include_directories(${boost_conversion_lib_SOURCE_DIR}/include)
#include_directories(${boost_throw_exception_lib_SOURCE_DIR}/include)
#include_directories(${boost_assert_lib_SOURCE_DIR}/include)
#include_directories(${boost_rational_lib_SOURCE_DIR}/include)
#include_directories(${boost_utility_lib_SOURCE_DIR}/include)
#include_directories(${boost_integer_lib_SOURCE_DIR}/include)
#include_directories(${boost_multiprecision_lib_SOURCE_DIR}/include)
#include_directories(${boost_math_lib_SOURCE_DIR}/include)
#include_directories(${boost_math_lib_SOURCE_DIR}/include_private)
#include_directories(${boost_detail_lib_SOURCE_DIR}/include)
#include_directories(${boost_type_index_lib_SOURCE_DIR}/include)
#include_directories(${boost_container_hash_lib_SOURCE_DIR}/include)
#include_directories(${boost_array_lib_SOURCE_DIR}/include)
#include_directories(${boost_function_types_lib_SOURCE_DIR}/include)
#include_directories(${boost_algorithm_lib_SOURCE_DIR}/include)
#include_directories(${boost_tuple_lib_SOURCE_DIR}/include)
#include_directories(${boost_qvm_lib_SOURCE_DIR}/include)
#include_directories(${boost_container_lib_SOURCE_DIR}/include)
#include_directories(${boost_intrusive_lib_SOURCE_DIR}/include)
#include_directories(${boost_smart_ptr_lib_SOURCE_DIR}/include)
#include_directories(${boost_lexical_cast_lib_SOURCE_DIR}/include)
#include_directories(${boost_tokenizer_lib_SOURCE_DIR}/include)
#include_directories(${boost_function_lib_SOURCE_DIR}/include)
#include_directories(${boost_bind_lib_SOURCE_DIR}/include)
#include_directories(${boost_parameter_lib_SOURCE_DIR}/include)
#include_directories(${boost_heap_lib_SOURCE_DIR}/include)
#include_directories(${boost_mp11_lib_SOURCE_DIR}/include)

add_library(boost_mp11 INTERFACE)
add_library(Boost::mp11 ALIAS boost_mp11)
target_include_directories(boost_mp11
        INTERFACE $<BUILD_INTERFACE:${boost_mp11_lib_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

add_library(boost_tokenizer INTERFACE)
add_library(Boost::tokenizer ALIAS boost_tokenizer)
target_include_directories(boost_tokenizer
        INTERFACE $<BUILD_INTERFACE:${boost_tokenizer_lib_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

add_library(boost_function INTERFACE)
add_library(Boost::function ALIAS boost_function)
target_include_directories(boost_function
        INTERFACE $<BUILD_INTERFACE:${boost_function_lib_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(boost_function INTERFACE boost_bind)

add_library(boost_bind INTERFACE)
add_library(Boost::bind ALIAS boost_bind)
target_include_directories(boost_bind
        INTERFACE $<BUILD_INTERFACE:${boost_bind_lib_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

add_library(boost_parameter INTERFACE)
add_library(Boost::parameter ALIAS boost_parameter)
target_include_directories(boost_parameter
        INTERFACE $<BUILD_INTERFACE:${boost_parameter_lib_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(boost_parameter INTERFACE boost_mp11)

add_library(boost_lexical_cast INTERFACE)
add_library(Boost::lexical_cast ALIAS boost_lexical_cast)
target_include_directories(boost_lexical_cast
        INTERFACE $<BUILD_INTERFACE:${boost_lexical_cast_lib_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

add_library(boost_tuple INTERFACE)
add_library(Boost::tuple ALIAS boost_tuple)
target_include_directories(boost_tuple
        INTERFACE $<BUILD_INTERFACE:${boost_tuple_lib_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

add_library(boost_qvm INTERFACE)
add_library(Boost::qvm ALIAS boost_qvm)
target_include_directories(boost_qvm
        INTERFACE $<BUILD_INTERFACE:${boost_qvm_lib_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

add_library(boost_container INTERFACE)
add_library(Boost::container ALIAS boost_container)
target_include_directories(boost_container
        INTERFACE $<BUILD_INTERFACE:${boost_container_lib_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(boost_container INTERFACE boost_intrusive)

add_library(boost_intrusive INTERFACE)
add_library(Boost::intrusive ALIAS boost_intrusive)
target_include_directories(boost_intrusive
        INTERFACE $<BUILD_INTERFACE:${boost_intrusive_lib_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

add_library(boost_smart_ptr INTERFACE)
add_library(Boost::smart_ptr ALIAS boost_smart_ptr)
target_include_directories(boost_smart_ptr
        INTERFACE $<BUILD_INTERFACE:${boost_smart_ptr_lib_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

add_library(boost_utility INTERFACE)
add_library(Boost::utility ALIAS boost_utility)
target_include_directories(boost_utility
        INTERFACE $<BUILD_INTERFACE:${boost_utility_lib_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

add_library(boost_integer INTERFACE)
add_library(Boost::integer ALIAS boost_integer)
target_include_directories(boost_integer
        INTERFACE $<BUILD_INTERFACE:${boost_integer_lib_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

add_library(boost_multiprecision INTERFACE)
add_library(Boost::multiprecision ALIAS boost_multiprecision)
target_include_directories(boost_multiprecision
        INTERFACE $<BUILD_INTERFACE:${boost_multiprecision_lib_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

add_library(boost_detail INTERFACE)
add_library(Boost::detail ALIAS boost_detail)
target_include_directories(boost_detail
        INTERFACE $<BUILD_INTERFACE:${boost_detail_lib_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

add_library(boost_type_index INTERFACE)
add_library(Boost::type_index ALIAS boost_type_index)
target_include_directories(boost_type_index
        INTERFACE $<BUILD_INTERFACE:${boost_type_index_lib_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(boost_type_index INTERFACE boost_container_hash)

add_library(boost_container_hash INTERFACE)
add_library(Boost::container_hash ALIAS boost_container_hash)
target_include_directories(boost_container_hash
        INTERFACE $<BUILD_INTERFACE:${boost_container_hash_lib_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

add_library(boost_array INTERFACE)
add_library(Boost::array ALIAS boost_array)
target_include_directories(boost_array
        INTERFACE $<BUILD_INTERFACE:${boost_array_lib_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

add_library(boost_function_types INTERFACE)
add_library(Boost::function_types ALIAS boost_function_types)
target_include_directories(boost_function_types
        INTERFACE $<BUILD_INTERFACE:${boost_function_types_lib_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

add_library(boost_algorithm INTERFACE)
add_library(Boost::algorithm ALIAS boost_algorithm)
target_include_directories(boost_algorithm
        INTERFACE $<BUILD_INTERFACE:${boost_algorithm_lib_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(boost_algorithm INTERFACE boost_function)

add_library(boost_rational INTERFACE)
add_library(Boost::rational ALIAS boost_rational)
target_include_directories(boost_rational
        INTERFACE $<BUILD_INTERFACE:${boost_rational_lib_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(boost_rational INTERFACE boost_utility boost_integer)

add_library(boost_assert INTERFACE)
add_library(Boost::assert ALIAS boost_assert)
target_include_directories(boost_assert
        INTERFACE $<BUILD_INTERFACE:${boost_assert_lib_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

add_library(boost_throw_exception INTERFACE)
add_library(Boost::throw_exception ALIAS boost_throw_exception)
target_include_directories(boost_throw_exception
        INTERFACE $<BUILD_INTERFACE:${boost_throw_exception_lib_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(boost_throw_exception INTERFACE boost_assert)

add_library(boost_concept_check INTERFACE)
add_library(Boost::concept_check ALIAS boost_concept_check)
target_include_directories(boost_concept_check
        INTERFACE $<BUILD_INTERFACE:${boost_concept_check_lib_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

add_library(boost_variant INTERFACE)
add_library(Boost::variant ALIAS boost_variant)
target_include_directories(boost_variant
        INTERFACE $<BUILD_INTERFACE:${boost_variant_lib_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(boost_variant INTERFACE boost_detail boost_type_index)

add_library(boost_move INTERFACE)
add_library(Boost::move ALIAS boost_move)
target_include_directories(boost_move
        INTERFACE $<BUILD_INTERFACE:${boost_move_lib_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

add_library(boost_core INTERFACE)
add_library(Boost::core ALIAS boost_core)
target_include_directories(boost_core
        INTERFACE $<BUILD_INTERFACE:${boost_core_lib_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

add_library(boost_conversion INTERFACE)
add_library(Boost::conversion ALIAS boost_conversion)
target_include_directories(boost_conversion
        INTERFACE $<BUILD_INTERFACE:${boost_conversion_lib_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(boost_conversion INTERFACE boost_concept_check boost_variant boost_move boost_core boost_throw_exception)

add_library(boost_static_assert INTERFACE)
add_library(Boost::static_assert ALIAS boost_static_assert)
target_include_directories(boost_static_assert
        INTERFACE $<BUILD_INTERFACE:${boost_static_assert_lib_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

add_library(boost_iterator INTERFACE)
add_library(Boost::iterator ALIAS boost_iterator)
target_include_directories(boost_iterator
        INTERFACE $<BUILD_INTERFACE:${boost_iterator_lib_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

add_library(boost_type_traits INTERFACE)
add_library(Boost::type_traits ALIAS boost_type_traits)
target_include_directories(boost_type_traits
        INTERFACE $<BUILD_INTERFACE:${boost_type_traits_lib_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

add_library(boost_mpl INTERFACE)
add_library(Boost::mpl ALIAS boost_mpl)
target_include_directories(boost_mpl
        INTERFACE $<BUILD_INTERFACE:${boost_mpl_lib_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

add_library(boost_preprocessor INTERFACE)
add_library(Boost::preprocessor ALIAS boost_preprocessor)
target_include_directories(boost_preprocessor
        INTERFACE $<BUILD_INTERFACE:${boost_preprocessor_lib_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

add_library(boost_range INTERFACE)
add_library(Boost::range ALIAS boost_range)
target_include_directories(boost_range
        INTERFACE $<BUILD_INTERFACE:${boost_range_lib_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(boost_range INTERFACE boost_mpl boost_type_traits boost_iterator)

add_library(boost_config INTERFACE)
add_library(Boost::config ALIAS boost_config)
target_include_directories(boost_config
        INTERFACE $<BUILD_INTERFACE:${boost_config_lib_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

add_library(boost_math INTERFACE)
add_library(Boost::math ALIAS boost_math)
target_include_directories(boost_math
        INTERFACE $<BUILD_INTERFACE:${boost_math_lib_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_compile_definitions(boost_math INTERFACE BOOST_MATH_STANDALONE=1)

add_library(boost_geometry INTERFACE)
add_library(Boost::geometry ALIAS boost_geometry)
target_include_directories(boost_geometry
        INTERFACE $<BUILD_INTERFACE:${boost_geometry_lib_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(boost_geometry INTERFACE boost_config boost_range boost_preprocessor boost_static_assert boost_conversion boost_rational boost_multiprecision boost_array boost_function_types boost_algorithm boost_tuple boost_qvm boost_container boost_smart_ptr boost_lexical_cast boost_tokenizer boost_math)

add_library(boost_heap INTERFACE)
add_library(Boost::heap ALIAS boost_heap)
target_include_directories(boost_heap
        INTERFACE $<BUILD_INTERFACE:${boost_heap_lib_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(boost_heap INTERFACE boost_parameter)