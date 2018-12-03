#include "easy_memmap.h"


// Constructor
MultiImagesMemmap::MultiImagesMemmap (string stream_name, string memmap_path) {
    // asign object vars
    name = stream_name;
    base_path = memmap_path;
    initialized = false;

    if (check_file(name)){
        memmap_file = init_memmap_r();
        labels_dict  = init_labels();
        initialized = true;
    }

}

json MultiImagesMemmap::init_labels(void){

    // read configuration file for labels names
    string labels;
    ifstream infile;
    infile.open(get_full_name() + "/" + LABELS_FILENAME);
    getline(infile,labels);
    infile.close();
    
    return json::parse(labels);
}


size_t MultiImagesMemmap::getFilesize(const char* filename) {
    struct stat st;
    stat(filename, &st);
    return st.st_size;
}

Mat MultiImagesMemmap::init_memmap_r(void){

    string settings = get_full_name() + "/" + CONFIG;
    size_t filesize = getFilesize(settings.c_str());

    //Open file
    int fd = open(settings.c_str(), O_RDONLY, 0);
    assert(fd != -1);
    // printf("Size of settings %lu \n", filesize);

    //Execute mmap
    short int* mmappedData = (short int *) mmap(NULL, filesize, PROT_READ, MAP_SHARED, fd, 0);
    assert(mmappedData != MAP_FAILED);
    
    // set object properties
    height = mmappedData[0];
    width = mmappedData[1];
    channels = mmappedData[2];

    // cout << "Image is "<< width << " x "  << height << " Number images: " << channels << endl << endl;

    //Cleanup
    int rc = munmap(mmappedData, filesize);
    assert(rc == 0);
    close(fd);

    string stream = get_full_name() + "/" + DATA;
    size_t filesize2 = getFilesize(stream.c_str());
    //Open file
    int fd2 = open(stream.c_str(), O_RDONLY , 0777);

    assert(fd2 != -1);
    // printf("Size %lu \n", filesize2);

    //Execute mmap
    unsigned char* mmappedData2 = (unsigned char *) mmap(NULL, filesize2, PROT_READ, MAP_SHARED, fd, 0);
    assert(mmappedData2 != MAP_FAILED);

    //Cleanup
    // int rc2 = munmap(mmappedData2, filesize2);
    // assert(rc2 == 0);
    close(fd2);

    // **** OPENCV STUFF *********** //
    return Mat(height, width, CV_8UC(channels), mmappedData2);
}


Mat MultiImagesMemmap::get_mat(void){
    return memmap_file;
}


// TODO: implement
bool MultiImagesMemmap::check_file(string name){
    vector<string> available_files = get_memmap_files();

    for(unsigned int i=0; i<available_files.size(); ++i){
        if (available_files[i].compare(name) == 0)
            return true;
    }
    cout << "File " << name << " not found." << endl;
    return false;
}

// TODO: implement
vector<string> MultiImagesMemmap::get_memmap_files(void)
{
    vector<string> r;

    DIR *dir = opendir(base_path.c_str());

    struct dirent *entry = readdir(dir);

    smatch match;
    regex re(PREFIX + string("_(.*)"));
    string result;

    while (entry != NULL)
    {
        if (entry->d_type == DT_DIR){

            string line(entry->d_name);

            if (regex_search( line, match, re) && match.size() > 1){ 
                result = match.str(1);
                r.push_back(result);
            }
        }

        entry = readdir(dir);
    }

    closedir(dir);
    return r;
}


string MultiImagesMemmap::get_full_name(void){
    return base_path + "/" + PREFIX + "_" + name;
}

Mat MultiImagesMemmap::get_image(int number){

    Mat image(height, width, CV_8UC3);

    Mat out[] = {image};
    int from_to[] = { number*3,0,  number*3+1,1,  number*3+2,2};

    mixChannels( &memmap_file, 1, out, 1, from_to, 3 );
    return out[0];
}

Mat MultiImagesMemmap::read(string key){
    Mat img;
    if (initialized){

        if (labels_dict.find(key) != labels_dict.end()){
            // printf("KEY FOUND\n");
            int index = labels_dict.at(key);
            return get_image(index);

        }
        else{
            printf("ERROR: KEY NOT FOUND\n");
        }
    }
    return img;

}

void MultiImagesMemmap::wait_until_available(void){
    while (!check_file(name)){
        sleep(2);
    }

    if (check_file(name) && !initialized){
        memmap_file = init_memmap_r();
        labels_dict  = init_labels();
        initialized = true;
    }

}


