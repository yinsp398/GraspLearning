#include "ConvertDB.h"
#include "MergeMean.h"
#include <stdio.h>

int main()
{

	convert_dataset("..\\ImageSet\\", "..\\ImageSet\\label.txt", "..\\ImageSet\\lmdb_test", "lmdb");
	addto_dataset("..\\ImageSet\\", "2mmCrossLine.png", 1, "..\\ImageSet\\lmdb_test", "lmdb");
	getimage_dataset("..\\ImageSet\\lmdb_test", "lmdb", "..\\ImageSet\\OutImg\\", "..\\ImageSet\\OutImg\\label.txt");
	std::cin.get();

	return 0;
}