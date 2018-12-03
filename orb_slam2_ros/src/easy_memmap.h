#ifndef EASY_MEMMAP_H
#define EASY_MEMMAP_H

#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <assert.h>
#include <stdio.h>
#include <dirent.h>
#include <regex>
#include "json.hpp"

#include "opencv2/opencv.hpp"
using namespace cv;

using namespace std;
using json = nlohmann::json;


size_t getFilesize(const char* filename) {
    struct stat st;
    stat(filename, &st);
    return st.st_size;
}


char* PREFIX = (char*) "easy_memmap";
char* CONFIG = (char*) "shape"; 
char* DATA = (char*) "data";
char* LABELS_FILENAME = (char*) "meta.json";

class MultiImagesMemmap {
    public:
        MultiImagesMemmap (string,string);
        Mat read(string key);
        vector<string> get_memmap_files(void);
        void wait_until_available(void);

    private:
        string name, base_path;
        json labels_dict;
        Mat memmap_file;
        int height, width, channels;
        bool initialized;

    private:
        string get_full_name(void);
        Mat init_memmap_r(void);
        bool check_file(string);
        Mat get_image(int);
        Mat get_mat(void);
        json init_labels(void);

};

#endif // UTILS_CONSOLE_H
