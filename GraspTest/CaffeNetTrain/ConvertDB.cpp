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
bool addto_dataset(const char* image_rootpath, const char* image_file, const int label, const char* db_path, const std::string& db_backend)
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
		return false;
	}

	// sequential
	std::string key_str = caffe::format_int(5, 8) + "_" + image_file;

	// Put in db
	std::string out;
	if (!datum.SerializeToString(&out))
	{
		std::cout << "datum serialize to string failed" << std::endl;
		return false;
	}
	txn->Put(key_str, out);

	// Commit db
	txn->Commit();
	db->Close();
}

unsigned int SaveConvert(const char* image_filepath, const char* label_filename, 
	const char* dbColor_path, const char* dbDepth_path, 
	const std::string& db_backend, unsigned int &FilePos, 
	const std::string &meancolorfile, const std::string &meandepthfile)
{
	const bool FLAGS_shuffle = true;
	const bool FLAGS_is_color = true;
	const bool FLAGS_encoded = true;
	const bool FLAGS_check_size = false;
	const std::string FLAGS_encode_type = "";
	const int FLAGS_resize_height = 0;
	const int FLAGS_resize_width = 0;

	std::ifstream infile(label_filename);
	std::vector<std::pair<std::string, int> > lines;
	std::string line;
	size_t pos;
	int label;
	infile.seekg(FilePos);
	while (std::getline(infile, line))
	{
		pos = line.find_last_of(' ');
		label = atoi(line.substr(pos + 1).c_str());
		lines.push_back(std::make_pair(line.substr(0, pos), label));
	}
	FilePos = infile.tellg();

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
	boost::scoped_ptr<caffe::db::DB> dbColor(caffe::db::GetDB(db_backend));
	dbColor->Open(dbColor_path, caffe::db::WRITE);
	boost::scoped_ptr<caffe::db::Transaction> txnColor(dbColor->NewTransaction());
	boost::scoped_ptr<caffe::db::DB> dbDepth(caffe::db::GetDB(db_backend));
	dbDepth->Open(dbDepth_path, caffe::db::WRITE);
	boost::scoped_ptr<caffe::db::Transaction> txnDepth(dbDepth->NewTransaction());

	// Storing to db
	std::string root_folder(image_filepath);
	caffe::Datum datumColor,datumDepth;
	unsigned int count = 0;
	int data_color_size = 0;
	int data_depth_size = 0;
	bool data_size_initialized = false;
	caffe::BlobProto sum_blob_color;
	caffe::BlobProto sum_blob_depth;

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
		status = ReadImageToDatum(root_folder + "Color\\" + lines[line_id].first + ".jpg",
			lines[line_id].second, resize_height, resize_width, FLAGS_is_color,
			enc, &datumColor);
		if (status == false)
			continue;

		//Check size
		if (!data_size_initialized)
		{
			data_color_size = datumColor.channels() * datumColor.height() * datumColor.width();
			sum_blob_color.set_num(1);
			sum_blob_color.set_channels(datumColor.channels());
			sum_blob_color.set_height(datumColor.height());
			sum_blob_color.set_width(datumColor.width());
			for (int i = 0; i < data_color_size; ++i) {
				sum_blob_color.add_data(0.);
			}
			data_depth_size = datumDepth.channels() * datumDepth.height() * datumDepth.width();
			sum_blob_depth.set_num(1);
			sum_blob_depth.set_channels(datumDepth.channels());
			sum_blob_depth.set_height(datumDepth.height());
			sum_blob_depth.set_width(datumDepth.width());
			for (int i = 0; i < data_depth_size; ++i) {
				sum_blob_depth.add_data(0.);
			}
			data_size_initialized = true;
		}
		else
		{
			CHECK_EQ(datumColor.data().size(), data_color_size) << "Incorrect data field size "
				<< datumColor.data().size();
			CHECK_EQ(datumDepth.data().size(), data_depth_size) << "Incorrect data field size "
				<< datumDepth.data().size();
		}
		//Get sum blob
		for (int i = 0; i < data_color_size; ++i)
		{
			sum_blob_color.set_data(i, sum_blob_color.data(i) + (uint8_t)datumColor.data()[i]);
		}
		for (int i = 0; i < data_depth_size; ++i)
		{
			sum_blob_depth.set_data(i, sum_blob_depth.data(i) + (uint8_t)datumDepth.data()[i]);
		}

		// sequential
		std::string key_str = caffe::format_int(line_id, 8) + "_" + lines[line_id].first;

		// Put in db
		std::string outColor;
		CHECK(datumColor.SerializeToString(&outColor));
		txnColor->Put(key_str, outColor);
		std::string outDepth;
		CHECK(datumDepth.SerializeToString(&outDepth));
		txnDepth->Put(key_str, outDepth);

		if (++count % 1000 == 0)
		{
			// Commit db
			txnColor->Commit();
			txnColor.reset(dbColor->NewTransaction());
			txnDepth->Commit();
			txnDepth.reset(dbDepth->NewTransaction());
			LOG(INFO) << "Processed " << count << " files.";
		}
	}
	// write the last batch
	if (count % 1000 != 0) 
	{
		txnColor->Commit();
		txnDepth->Commit();
		LOG(INFO) << "Processed " << count << " files.";
	}
	for (int i = 0; i < sum_blob_color.data_size(); ++i) {
		sum_blob_color.set_data(i, sum_blob_color.data(i) / count);
	}
	WriteProtoToBinaryFile(sum_blob_color, meancolorfile);
	for (int i = 0; i < sum_blob_depth.data_size(); ++i) {
		sum_blob_depth.set_data(i, sum_blob_depth.data(i) / count);
	}
	WriteProtoToBinaryFile(sum_blob_depth, meandepthfile);
	return count;
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
		//use caffe function decodedatumtocvmatnative replace my function datumtomat
		Img = caffe::DecodeDatumToCVMatNative(datum);
		//DatumToMat(&datum, Img);
		cv::imwrite(image_filepath + key_str, Img);
		cursor_->Next();
	}
	std::cout << "have get the end of dataset" << std::endl;
	labelfp.close();
}

