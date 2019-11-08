#ifndef ZIP_MZ_HPP
#define ZIP_MZ_HPP

#include <cassert>
#include <cerrno>
#include <cstring>
#include <functional>
#include <memory>
#include <string>

#include "miniz.h"

namespace miniz {

class zip_reader {
private:
  mz_zip_archive zip_archive;

	// copy constructor disabled.
  zip_reader(const zip_reader&) = delete;
  void operator=(const zip_reader&) = delete;

public:
	/**
	 * Open an archive on the disk.
	 *
	 * \param path the path
	 * \param flags the optional flags
	 * \throw std::runtime_error on errors
	 */
  zip_reader(const std::string& path)
	{
    mz_zip_zero_struct(&zip_archive);
    mz_bool status = mz_zip_reader_init_file(&zip_archive, path.c_str(), 0);

    if (!status) {
      throw std::runtime_error(mz_zip_get_error_string(zip_archive.m_last_error));
    }
	}

  ~zip_reader()
  {
    mz_zip_reader_end(&zip_archive);
  }

	/**
	 * Move constructor defaulted.
	 *
	 * \param other the other archive
	 */
  zip_reader(zip_reader&& other) noexcept = default;

	/**
	 * Move operator defaulted.
	 *
	 * \param other the other archive
	 * \return *this
	 */
  zip_reader& operator=(zip_reader&& other) noexcept  = default;

  std::string read(const std::string &file_name)
  {
    int index = mz_zip_reader_locate_file(&zip_archive, file_name.c_str(), 0, 0);
    if (index < 0)
      throw std::runtime_error("File not found");

    mz_zip_archive_file_stat file_stat;
    mz_zip_reader_file_stat(&zip_archive, index, &file_stat);

    std::string data;
    data.resize(file_stat.m_uncomp_size);
    mz_zip_reader_extract_to_mem(&zip_archive, index, &(data[0]), data.size(), 0);

    return data;
  }

};

class zip_writer {
private:
  mz_zip_archive zip_archive;

  // copy constructor disabled.
  zip_writer(const zip_writer&) = delete;
  void operator=(const zip_writer&) = delete;

public:
  /**
   * Open an archive on the disk.
   *
   * \param path the path
   * \param flags the optional flags
   * \throw std::runtime_error on errors
   */
  zip_writer(const std::string& path)
  {
    remove(path.c_str());
    mz_zip_zero_struct(&zip_archive);
    mz_bool status = mz_zip_writer_init_file(&zip_archive, path.c_str(), 0);

    if (!status) {
      throw std::runtime_error(mz_zip_get_error_string(zip_archive.m_last_error));
    }
  }

  ~zip_writer()
  {
    mz_zip_writer_finalize_archive(&zip_archive);
    mz_zip_writer_end(&zip_archive);
  }

  mz_bool add(const std::string &file_name, const std::string &data)
  {
    return mz_zip_writer_add_mem(&zip_archive, file_name.c_str(), data.data(), data.size(), 0);
  }
};

} // !miniz

#endif // !ZIP_MZ_HPP
