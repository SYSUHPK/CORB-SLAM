//
// Created by lifu on 17-2-1.
//
// 新增
#include "System.h"
#include "Cache.h"
#include "LightKeyFrame.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>
// 轻量级的关键帧
namespace ORB_SLAM2 {

    LightKeyFrame::LightKeyFrame() : mnId(0), mpCache(nullptr)  {

    }

    LightKeyFrame::LightKeyFrame(long unsigned int pId, Cache *pCache)
            : mnId(pId), mpCache(pCache) {

    }

    LightKeyFrame::LightKeyFrame(KeyFrame *pKF) {
        if( pKF ) {
            this->mnId = pKF->mnId;
            this->mpCache = pKF->getCache();
        } else{
            this->mnId = 0;
            this->mpCache = nullptr;
        }
    }

    KeyFrame* LightKeyFrame::getKeyFrame() const{

        if( this->mpCache )
            return this->mpCache->getKeyFrameById( this->mnId );
        else
            return nullptr;

    }

    KeyFrame* LightKeyFrame::getKeyFrameInCache() {

        if( this->mpCache ) {
            if( this->mpCache->KeyFrameInCache( this->mnId)) {
                return this->mpCache->getKeyFrameById( this->mnId );
            }
        }

        return nullptr;

    }



}