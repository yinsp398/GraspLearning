convert_imageset-d.exe ..\ImageSet\CloudPoints\ ..\ImageSet\train.txt ..\ImageSet\lmdbdepth -gray=1 -backend=lmdb
convert_imageset-d.exe ..\ImageSet\CloudPoints\ ..\ImageSet\test.txt ..\ImageSet\lmdbdepthtest -gray=1 -backend=lmdb
convert_imageset-d.exe ..\ImageSet\Color\ ..\ImageSet\train.txt ..\ImageSet\lmdbcolor -gray=1 -backend=lmdb
convert_imageset-d.exe ..\ImageSet\Color\ ..\ImageSet\test.txt ..\ImageSet\lmdbcolortest -gray=1 -backend=lmdb
compute_image_mean-d.exe ..\ImageSet\lmdbcolor ..\ImageSet\MeanFile\meancolor.binaryproto --backend=lmdb
compute_image_mean-d.exe ..\ImageSet\lmdbdepth ..\ImageSet\MeanFile\meandepth.binaryproto --backend=lmdb