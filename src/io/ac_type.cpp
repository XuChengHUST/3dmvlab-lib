#include "io/ac_type.h"
#include <cstdio>
#include <cstring>

namespace pc {
  ACFileType::ACFileType(const std::string& filename)
    : numVerts_(-1), tableX_(-1), tableY_(-1), numHeaderByte_(-1),
      pVertex_(nullptr), pOrder_(nullptr) {
        std::FILE *fp;

        fp = fopen(filename.c_str(), "r"); //read AC file ascii
        if (fp == NULL) {
          std::cout << filename << " open failed!" << '\n';
          return;
        }
        //read AC head info
        char headerTemp[50];
        if(NULL == fgets(headerTemp, 50 ,fp))
          return;
        while (strncmp(headerTemp,"FIRST_PIXEL_TABLE_BYTE",22)!=0) {
            if(!strncmp(headerTemp,"NUMBER_OF_POINTS",16)) {
                numVerts_ = atoi(headerTemp+16); //get vertex numbers
            }
            if (!strncmp(headerTemp,"FIRST_DATA_BYTE",15)) {
                numHeaderByte_ = atoi(headerTemp+15); //get first data byte
            }
            if (!strncmp(headerTemp,"PIXEL_TABLE_MAX_X",17)) {
                tableX_ = atoi(headerTemp+17); //get table col limits
            }
            if (!strncmp(headerTemp,"PIXEL_TABLE_MAX_Y",17)) {
                tableY_ = atoi(headerTemp+17); //get table row limits
            }
            memset(headerTemp,0,50); //clear headerTemp
            if(NULL == fgets(headerTemp, 50 ,fp))
              return;
        }
        fclose(fp);
        int orderbyte = numHeaderByte_ + 12 * numVerts_;

        fp = fopen(filename.c_str(), "rb"); //reopen file binary
        if (fp == NULL) {
          std::cout << "AC file open failed!" << std::endl;
          return;
        }

        pVertex_ = new float[3*numVerts_]; // allocate memory for vertex
        pOrder_  = new short[2*numVerts_]; // index info

        fseek(fp, numHeaderByte_, SEEK_SET); //load 3D points
        size_t vertex_size = 3 * numVerts_;
        if(vertex_size != fread(pVertex_, sizeof(float), vertex_size, fp))
          return;
        fseek(fp, orderbyte, SEEK_SET); //load order index
        size_t order_size = 2 * numVerts_;
        if(order_size != fread(pOrder_, sizeof(unsigned short int), order_size, fp))
          return;
        fclose(fp);
  }
  ACFileType::~ACFileType() {
      //release memory
      delete[] pVertex_;
      delete[] pOrder_;
  }

}
