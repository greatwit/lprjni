#include <jni.h>
#include <string>

#include "include/Pipeline.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp> 


void conv_yuv420_to_mat(cv::Mat &dst, unsigned char* pYUV420, int width, int height)
{
	if (!pYUV420) {
		return;
	}
 
	IplImage *yuvimage, *rgbimg, *yimg, *uimg, *vimg, *uuimg, *vvimg;
 
	int nWidth = width;
	int nHeight = height;
	rgbimg = cvCreateImage(cvSize(nWidth, nHeight), IPL_DEPTH_8U, 3);
	yuvimage = cvCreateImage(cvSize(nWidth, nHeight), IPL_DEPTH_8U, 3);
 
	yimg = cvCreateImageHeader(cvSize(nWidth, nHeight), IPL_DEPTH_8U, 1);
	uimg = cvCreateImageHeader(cvSize(nWidth / 2, nHeight / 2), IPL_DEPTH_8U, 1);
	vimg = cvCreateImageHeader(cvSize(nWidth / 2, nHeight / 2), IPL_DEPTH_8U, 1);
 
	uuimg = cvCreateImage(cvSize(nWidth, nHeight), IPL_DEPTH_8U, 1);
	vvimg = cvCreateImage(cvSize(nWidth, nHeight), IPL_DEPTH_8U, 1);
 
	cvSetData(yimg, pYUV420, nWidth);
	cvSetData(uimg, pYUV420 + nWidth * nHeight, nWidth / 2);
	cvSetData(vimg, pYUV420 + long(nWidth*nHeight*1.25), nWidth / 2);
	cvResize(uimg, uuimg, CV_INTER_LINEAR);
	cvResize(vimg, vvimg, CV_INTER_LINEAR);
 
	cvMerge(yimg, uuimg, vvimg, NULL, yuvimage);
	cvCvtColor(yuvimage, rgbimg, CV_YCrCb2RGB);
 
	cvReleaseImage(&uuimg);
	cvReleaseImage(&vvimg);
	cvReleaseImageHeader(&yimg);
	cvReleaseImageHeader(&uimg);
	cvReleaseImageHeader(&vimg);
 
	cvReleaseImage(&yuvimage);
 
	//dst = Mat(*rgbimg,int(1));
	//dst = cvarrToMat(rgbimg, true);
	//rgbimg->
	cvReleaseImage(&rgbimg);
}


std::string jstring2str(JNIEnv* env, jstring jstr)
{
    char*   rtn   =   NULL;
    jclass   clsstring   =   env->FindClass("java/lang/String");
    jstring   strencode   =   env->NewStringUTF("GB2312");
    jmethodID   mid   =   env->GetMethodID(clsstring,   "getBytes",   "(Ljava/lang/String;)[B");
    jbyteArray   barr=   (jbyteArray)env->CallObjectMethod(jstr,mid,strencode);
    jsize   alen   =   env->GetArrayLength(barr);
    jbyte*   ba   =   env->GetByteArrayElements(barr,JNI_FALSE);
    if(alen   >   0)
    {
        rtn   =   (char*)malloc(alen+1);
        memcpy(rtn,ba,alen);
        rtn[alen]=0;
    }
    env->ReleaseByteArrayElements(barr,ba,0);
    std::string stemp(rtn);
    free(rtn);
    return   stemp;
}


extern "C" {

JNIEXPORT jlong JNICALL
Java_com_hdalpr_PlateRecognition_InitPlateRecognizer(
        JNIEnv *env, jobject obj,
        jstring detector_filename,
        jstring finemapping_prototxt, jstring finemapping_caffemodel,
        jstring segmentation_prototxt, jstring segmentation_caffemodel,
        jstring charRecognization_proto, jstring charRecognization_caffemodel) {

    std::string detector_path = jstring2str(env, detector_filename);
    std::string finemapping_prototxt_path = jstring2str(env, finemapping_prototxt);
    std::string finemapping_caffemodel_path = jstring2str(env, finemapping_caffemodel);
    std::string segmentation_prototxt_path = jstring2str(env, segmentation_prototxt);
    std::string segmentation_caffemodel_path = jstring2str(env, segmentation_caffemodel);
    std::string charRecognization_proto_path = jstring2str(env, charRecognization_proto);
    std::string charRecognization_caffemodel_path = jstring2str(env, charRecognization_caffemodel);


    pr::PipelinePR *PR = new pr::PipelinePR(detector_path,
                                            finemapping_prototxt_path, finemapping_caffemodel_path,
                                            segmentation_prototxt_path, segmentation_caffemodel_path,
                                            charRecognization_proto_path, charRecognization_caffemodel_path);

    return (jlong) PR;
}

JNIEXPORT jstring JNICALL
Java_com_hdalpr_PlateRecognition_SimpleRecognization(
        JNIEnv *env, jobject obj,
        jlong matPtr, jlong object_pr) {
    pr::PipelinePR *PR = (pr::PipelinePR *) object_pr;
    cv::Mat &mRgb = *(cv::Mat *) matPtr;
    cv::Mat rgb;
//    cv::cvtColor(mRgb,rgb,cv::COLOR_RGBA2GRAY);
    cv::cvtColor(mRgb,rgb,cv::COLOR_RGBA2BGR);

//    cv::imwrite("/sdcard/demo.jpg",rgb);

    //1表示SEGMENTATION_BASED_METHOD在方法里有说明
    std::vector<pr::PlateInfo> list_res= PR->RunPiplineAsImage(rgb, 0);
//    std::vector<pr::PlateInfo> list_res= PR->RunPiplineAsImage(rgb,1);
    std::string concat_results;
    for(auto one:list_res)
    {
        if (one.confidence>0.8) {
		char str[20]= {0};
	
        	cv::Rect region = one.getPlateRect();
		sprintf(str, "%d,%d,%d,%d,", region.x, region.y, region.width, region.height);
            	concat_results+=one.getPlateName()+","+str;
            	//concat_results+=os+",";
            	//concat_results+=region.y+",";
            	//concat_results+=region.width+",";
            	//concat_results+=region.height+",";
        }
    }
    concat_results = concat_results.substr(0,concat_results.size()-1);

    return env->NewStringUTF(concat_results.c_str());
}

JNIEXPORT void JNICALL
Java_com_hdalpr_PlateRecognition_ReleasePlateRecognizer(
        JNIEnv *env, jobject obj,
        jlong object_re) {
//    std::string hello = "Hello from C++";
    pr::PipelinePR *PR = (pr::PipelinePR *) object_re;
    delete PR;
}

}


