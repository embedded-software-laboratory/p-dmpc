#include "../include/ArrayTypePrinting.h"

std::ostream& operator<<(std::ostream& stream, matlab::data::ArrayType const& arrayType) {
	switch (arrayType) {
		case matlab::data::ArrayType::LOGICAL: stream << "LOGICAL"; break;
		case matlab::data::ArrayType::CHAR: stream << "CHAR"; break;
		case matlab::data::ArrayType::MATLAB_STRING: stream << "MATLAB_STRING"; break;
		case matlab::data::ArrayType::DOUBLE: stream << "DOUBLE"; break;
		case matlab::data::ArrayType::SINGLE: stream << "SINGLE"; break;
		case matlab::data::ArrayType::INT8: stream << "INT8"; break;
		case matlab::data::ArrayType::UINT8: stream << "UINT8"; break;
		case matlab::data::ArrayType::INT16: stream << "INT16"; break;
		case matlab::data::ArrayType::UINT16: stream << "UINT16"; break;
		case matlab::data::ArrayType::INT32: stream << "INT32"; break;
		case matlab::data::ArrayType::UINT32: stream << "UINT32"; break;
		case matlab::data::ArrayType::INT64: stream << "INT64"; break;
		case matlab::data::ArrayType::UINT64: stream << "UINT64"; break;
		case matlab::data::ArrayType::COMPLEX_DOUBLE: stream << "COMPLEX_DOUBLE"; break;
		case matlab::data::ArrayType::COMPLEX_SINGLE: stream << "COMPLEX_SINGLE"; break;
		case matlab::data::ArrayType::COMPLEX_INT8: stream << "COMPLEX_INT8"; break;
		case matlab::data::ArrayType::COMPLEX_UINT8: stream << "COMPLEX_UINT8"; break;
		case matlab::data::ArrayType::COMPLEX_INT16: stream << "COMPLEX_INT16"; break;
		case matlab::data::ArrayType::COMPLEX_UINT16: stream << "COMPLEX_UINT16"; break;
		case matlab::data::ArrayType::COMPLEX_INT32: stream << "COMPLEX_INT32"; break;
		case matlab::data::ArrayType::COMPLEX_UINT32: stream << "COMPLEX_UINT32"; break;
		case matlab::data::ArrayType::COMPLEX_INT64: stream << "COMPLEX_INT64"; break;
		case matlab::data::ArrayType::COMPLEX_UINT64: stream << "COMPLEX_UINT64"; break;
		case matlab::data::ArrayType::CELL: stream << "CELL"; break;
		case matlab::data::ArrayType::STRUCT: stream << "STRUCT"; break;
		case matlab::data::ArrayType::OBJECT: stream << "OBJECT"; break;
		case matlab::data::ArrayType::VALUE_OBJECT: stream << "VALUE_OBJECT"; break;
		case matlab::data::ArrayType::HANDLE_OBJECT_REF: stream << "HANDLE_OBJECT_REF"; break;
		case matlab::data::ArrayType::ENUM: stream << "ENUM"; break;
		case matlab::data::ArrayType::SPARSE_LOGICAL: stream << "SPARSE_LOGICAL"; break;
		case matlab::data::ArrayType::SPARSE_DOUBLE: stream << "SPARSE_DOUBLE"; break;
		case matlab::data::ArrayType::SPARSE_COMPLEX_DOUBLE: stream << "SPARSE_COMPLEX_DOUBLE"; break;
		case matlab::data::ArrayType::UNKNOWN: stream << "UNKNOWN"; break;
		default: throw std::invalid_argument("ArrayType has impossible State!");
	}

	return stream;
}