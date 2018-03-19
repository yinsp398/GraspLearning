#include "ConvertDB.h"
#include <stdio.h>
#include <string>
#include <fstream>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <google/protobuf/text_format.h>

#if defined(USE_LEVELDB) && defined(USE_LMDB)
#include <leveldb/db.h>
#include <leveldb/write_batch.h>
#include <lmdb.h>
#endif

#if defined(_MSC_VER)
#include <direct.h>
#define mkdir(X, Y) _mkdir(X)
#endif

#include "boost/scoped_ptr.hpp"
#include "caffe/proto/caffe.pb.h"
#include "caffe/util/db.hpp"
#include "caffe/util/format.hpp"
#include "caffe/util/io.hpp"
#include "caffe/util/rng.hpp"


#include "caffe/data_transformer.hpp"
#include "caffe/util/benchmark.hpp"

#include <opencv2\opencv.hpp>

uint32_t swap_endian(uint32_t val) {
	val = ((val << 8) & 0xFF00FF00) | ((val >> 8) & 0xFF00FF);
	return (val << 16) | (val >> 16);
}
void addto_dataset(const char* image_rootpath, const char* image_file, const int label, const char* db_path, const std::string& db_backend)
{
	// Create new DB
	boost::scoped_ptr<caffe::db::DB> db(caffe::db::GetDB(db_backend));
	db->Open(db_path, caffe::db::WRITE);
	boost::scoped_ptr<caffe::db::Transaction> txn(db->NewTransaction());

	

	caffe::Datum datum; 
	std::string imagepath(image_rootpath);
	bool status = ReadImageToDatum(imagepath + image_file,
		label, 0, 0, true,
		"", &datum);
	if (!status)
	{
		std::cout << "add image to dataset failed" << std::endl;
		db->Close();
		return;
	}

	// sequential
	std::string key_str = caffe::format_int(5, 8) + "_" + image_file;

	// Put in db
	std::string out;
	CHECK(datum.SerializeToString(&out));
	txn->Put(key_str, out);

	// Commit db
	txn->Commit();
	db->Close();
}

void convert_dataset(const char* image_filepath, const char* label_filename, const char* db_path, const std::string& db_backend)
{
	const bool FLAGS_shuffle = false;
	const bool FLAGS_is_color = true;
	const bool FLAGS_encoded = false;
	const bool FLAGS_check_size = false;
	const std::string FLAGS_encode_type = "";
	const int FLAGS_resize_height = 0;
	const int FLAGS_resize_width = 0;

	std::ifstream infile(label_filename);
	std::vector<std::pair<std::string, int> > lines;
	std::string line;
	size_t pos;
	int label;
	while (std::getline(infile, line)) 
	{
		pos = line.find_last_of(' ');
		label = atoi(line.substr(pos + 1).c_str());
		lines.push_back(std::make_pair(line.substr(0, pos), label));
	}

	if (FLAGS_shuffle) 
	{
		// randomly shuffle data
		LOG(INFO) << "Shuffling data";
		caffe::shuffle(lines.begin(), lines.end());
	}
	LOG(INFO) << "A total of " << lines.size() << " images.";

	if (FLAGS_encode_type.size() && !FLAGS_encoded)
		LOG(INFO) << "encode_type specified, assuming encoded=true.";

	int resize_height = std::max<int>(0, FLAGS_resize_height);
	int resize_width = std::max<int>(0, FLAGS_resize_width);

	// Create new DB
	boost::scoped_ptr<caffe::db::DB> db(caffe::db::GetDB(db_backend));
	db->Open(db_path, caffe::db::NEW);
	boost::scoped_ptr<caffe::db::Transaction> txn(db->NewTransaction());

	// Storing to db
	std::string root_folder(image_filepath);
	caffe::Datum datum;
	int count = 0;
	int data_size = 0;
	bool data_size_initialized = false;

	for (int line_id = 0; line_id < lines.size(); ++line_id)
	{
		bool status;
		std::string enc = FLAGS_encode_type;
		if (FLAGS_encoded && !enc.size())
		{
			// Guess the encoding type from the file name
			std::string fn = lines[line_id].first;
			size_t p = fn.rfind('.');
			if (p == fn.npos)
				LOG(WARNING) << "Failed to guess the encoding of '" << fn << "'";
			enc = fn.substr(p);
			std::transform(enc.begin(), enc.end(), enc.begin(), ::tolower);
		}
		status = ReadImageToDatum(root_folder + lines[line_id].first,
			lines[line_id].second, resize_height, resize_width, FLAGS_is_color,
			enc, &datum);
		if (status == false)
			continue;
		if (FLAGS_check_size)
		{
			if (!data_size_initialized)
			{
				data_size = datum.channels() * datum.height() * datum.width();
				data_size_initialized = true;
			}
			else
			{
				const std::string& data = datum.data();
				CHECK_EQ(data.size(), data_size) << "Incorrect data field size "
					<< data.size();
			}
		}
		// sequential
		std::string key_str = caffe::format_int(line_id, 8) + "_" + lines[line_id].first;

		// Put in db
		std::string out;
		CHECK(datum.SerializeToString(&out));
		txn->Put(key_str, out);

		if (++count % 1000 == 0) 
		{
			// Commit db
			txn->Commit();
			txn.reset(db->NewTransaction());
			LOG(INFO) << "Processed " << count << " files.";
		}
	}
	// write the last batch
	if (count % 1000 != 0) {
		txn->Commit();
		LOG(INFO) << "Processed " << count << " files.";
	}
}

void DatumToMat(const caffe::Datum* datum, cv::Mat &cv_img)
{
	int datum_channels = datum->channels();
	int datum_height = datum->height();
	int datum_width = datum->width();
	int datum_size = datum_channels * datum_height * datum_width;

	std::string buffer(datum_size, ' ');
	buffer = datum->data();

	for (int h = 0; h < datum_height; ++h) 
	{
		uchar* ptr = cv_img.ptr<uchar>(h);
		int img_index = 0;
		for (int w = 0; w < datum_width; ++w) 
		{
			for (int c = 0; c < datum_channels; ++c) 
			{
				int datum_index = (c * datum_height + h) * datum_width + w;
				ptr[img_index++] = static_cast<uchar>(buffer[datum_index]);
			}
		}
	}
}

void getimage_dataset(const char* dp_path, const char* db_backend, const char* image_filepath, const char* labelpath)
{
	boost::shared_ptr<caffe::db::DB> db_;
	boost::shared_ptr<caffe::db::Cursor> cursor_;

	//Initialize the DB dataset and iteration cursor
	db_.reset(caffe::db::GetDB(db_backend));
	db_->Open(dp_path, caffe::db::READ);
	cursor_.reset(db_->NewCursor());

	std::ofstream labelfp;
	labelfp.open(labelpath, std::ios::out);

	caffe::Datum datum;
	while (cursor_->valid())
	{
		//Get key and value
		std::string key_str = cursor_->key();
		datum.ParseFromString(cursor_->value());

		labelfp << key_str << " " << datum.label() << "\n";

		cv::Mat Img(datum.height(), datum.width(), CV_8UC3);
		DatumToMat(&datum, Img);
		cv::imwrite(image_filepath + key_str, Img);
		cursor_->Next();
	}
	std::cout << "have get the end of dataset" << std::endl;
	labelfp.close();
}

