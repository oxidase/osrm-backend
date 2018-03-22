#include "storage/tar.hpp"

#include "../common/range_tools.hpp"
#include "../common/temporary_file.hpp"

#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_SUITE(tar)

using namespace osrm;

BOOST_AUTO_TEST_CASE(list_tar_file)
{
    storage::tar::FileReader reader(TEST_DATA_DIR "/tar_test.tar",
                                    storage::tar::FileReader::HasNoFingerprint);

    std::vector<storage::tar::FileReader::TarEntry> file_list;
    reader.List(std::back_inserter(file_list));

    auto reference_0 = storage::tar::FileReader::TarEntry{"foo_1.txt", 4};
    auto reference_1 = storage::tar::FileReader::TarEntry{"bla/foo_2.txt", 4};
    auto reference_2 = storage::tar::FileReader::TarEntry{"foo_3.txt", 4};

    BOOST_CHECK_EQUAL(std::get<0>(file_list[0]), std::get<0>(reference_0));
    BOOST_CHECK_EQUAL(std::get<1>(file_list[0]), std::get<1>(reference_0));
    BOOST_CHECK_EQUAL(std::get<0>(file_list[1]), std::get<0>(reference_1));
    BOOST_CHECK_EQUAL(std::get<1>(file_list[1]), std::get<1>(reference_1));
    BOOST_CHECK_EQUAL(std::get<0>(file_list[2]), std::get<0>(reference_2));
    BOOST_CHECK_EQUAL(std::get<1>(file_list[2]), std::get<1>(reference_2));
}

BOOST_AUTO_TEST_CASE(read_tar_file)
{
    storage::tar::FileReader reader(TEST_DATA_DIR "/tar_test.tar",
                                    storage::tar::FileReader::HasNoFingerprint);

    char result_0[4];
    reader.ReadInto("foo_1.txt", result_0, 4);

    // Note: This is an out-of-order read, foo_3 comes after bla/foo_2 in the tar file
    char result_1[4];
    reader.ReadInto("foo_3.txt", result_1, 4);

    char result_2[4];
    reader.ReadInto("bla/foo_2.txt", result_2, 4);

    BOOST_CHECK_EQUAL(std::string(result_0, 4), std::string("bla\n"));
    BOOST_CHECK_EQUAL(std::string(result_1, 4), std::string("foo\n"));
    BOOST_CHECK_EQUAL(std::string(result_2, 4), std::string("baz\n"));
}

BOOST_AUTO_TEST_CASE(write_tar_file)
{
    TemporaryFile tmp{TEST_DATA_DIR "/tar_write_test.tar"};

    std::uint64_t single_64bit_integer = 0xDEADBEEFAABBCCDD;
    std::uint32_t single_32bit_integer = 0xDEADBEEF;

    std::vector<std::uint32_t> vector_32bit = {0, 1, 2, 3, 4, 1 << 30, 0, 1 << 22, 0xFFFFFFFF};
    std::vector<std::uint64_t> vector_64bit = {
        0, 1, 2, 3, 4, 1ULL << 62, 0, 1 << 22, 0xFFFFFFFFFFFFFFFF};

    {
        storage::tar::FileWriter writer(tmp.path, storage::tar::FileWriter::GenerateFingerprint);
        writer.WriteFrom("foo/single_64bit_integer", single_64bit_integer);
        writer.WriteFrom("bar/single_32bit_integer", single_32bit_integer);
        writer.WriteElementCount64("baz/bla/64bit_vector", vector_64bit.size());
        writer.WriteFrom("baz/bla/64bit_vector", vector_64bit.data(), vector_64bit.size());
        writer.WriteElementCount64("32bit_vector", vector_32bit.size());
        writer.WriteFrom("32bit_vector", vector_32bit.data(), vector_32bit.size());
    }

    storage::tar::FileReader reader(tmp.path, storage::tar::FileReader::VerifyFingerprint);

    std::uint32_t result_32bit_integer;
    std::uint64_t result_64bit_integer;
    reader.ReadInto("bar/single_32bit_integer", result_32bit_integer);
    reader.ReadInto("foo/single_64bit_integer", result_64bit_integer);
    BOOST_CHECK_EQUAL(result_32bit_integer, single_32bit_integer);
    BOOST_CHECK_EQUAL(result_64bit_integer, single_64bit_integer);

    std::vector<std::uint64_t> result_64bit_vector(
        reader.ReadElementCount64("baz/bla/64bit_vector"));
    reader.ReadInto("baz/bla/64bit_vector", result_64bit_vector.data(), result_64bit_vector.size());
    std::vector<std::uint32_t> result_32bit_vector(reader.ReadElementCount64("32bit_vector"));
    reader.ReadInto("32bit_vector", result_32bit_vector.data(), result_32bit_vector.size());

    CHECK_EQUAL_COLLECTIONS(result_64bit_vector, vector_64bit);
    CHECK_EQUAL_COLLECTIONS(result_32bit_vector, vector_32bit);
}

BOOST_AUTO_TEST_CASE(continue_write_tar_file)
{
    TemporaryFile tmp{TEST_DATA_DIR "/tar_continue_write_test.tar"};

    // more than 64 values to ensure we fill up more than one tar block of 512 bytes
    std::vector<std::uint64_t> vector_64bit = {
        0, 1, 2, 3, 4, 1ULL << 62, 0, 1 << 22, 0xFFFFFFFFFFFFFFFF, 0xFF00FF0000FF00FF, 11, 12, 13, 14, 15, 16,
        17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32,
        33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48,
        49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64,
        65, 66, 67, 68, 69, 70
    };

    {
        storage::tar::FileWriter writer(tmp.path, storage::tar::FileWriter::GenerateFingerprint);
        writer.WriteElementCount64("baz/bla/64bit_vector", vector_64bit.size());
        writer.WriteFrom("baz/bla/64bit_vector", vector_64bit.data(), 12);
        writer.ContinueFrom("baz/bla/64bit_vector", vector_64bit.data() + 12, 30);
        writer.ContinueFrom("baz/bla/64bit_vector", vector_64bit.data() + 42, 10);
        writer.ContinueFrom("baz/bla/64bit_vector", vector_64bit.data() + 52, vector_64bit.size() - 52);
    }

    storage::tar::FileReader reader(tmp.path, storage::tar::FileReader::VerifyFingerprint);

    std::vector<std::uint64_t> result_64bit_vector(
        reader.ReadElementCount64("baz/bla/64bit_vector"));
    reader.ReadInto("baz/bla/64bit_vector", result_64bit_vector.data(), result_64bit_vector.size());

    CHECK_EQUAL_COLLECTIONS(result_64bit_vector, vector_64bit);
}

BOOST_AUTO_TEST_SUITE_END()
