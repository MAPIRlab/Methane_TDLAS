/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------
 
  File:        FeatureContainer.cpp

  Description: Implementation of class VmbCPP::FeatureContainer.

-------------------------------------------------------------------------------

  THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR IMPLIED
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF TITLE,
  NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A PARTICULAR  PURPOSE ARE
  DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, 
  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED  
  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR 
  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

=============================================================================*/

#include <map>
#include <string>

#include <VmbCPP/FeatureContainer.h>

#include <VmbCPP/VmbSystem.h>


namespace VmbCPP {

typedef std::map<std::string, FeaturePtr> FeaturePtrMap;

struct FeatureContainer::Impl
{
    VmbHandle_t         m_handle;

    bool                m_bAllFeaturesFetched;

    FeaturePtrMap       m_features;
};

FeatureContainer::FeatureContainer()
    :   m_pImpl ( new Impl() )
{
    m_pImpl->m_bAllFeaturesFetched = false;
    m_pImpl->m_handle = nullptr;
}

FeatureContainer::~FeatureContainer()
{
    Reset();
    RevokeHandle();
}

VmbErrorType FeatureContainer::GetFeatureByName( const char *name, FeaturePtr &rFeature )
{
    VmbError_t res;

    if ( nullptr == name )
    {
        return VmbErrorBadParameter;
    }

    if ( nullptr == m_pImpl->m_handle )
    {
        return VmbErrorDeviceNotOpen;
    }

    FeaturePtrMap::iterator iter = m_pImpl->m_features.find( name );
    if ( iter != m_pImpl->m_features.end() )
    {
        rFeature = iter->second;
        return VmbErrorSuccess;
    }

    VmbFeatureInfo_t featureInfo;
        
    res = VmbFeatureInfoQuery(m_pImpl->m_handle, name, &featureInfo, sizeof(VmbFeatureInfo_t));

    if ( VmbErrorSuccess == res )
    {

        try
        {
            FeaturePtr tmpFeature(new Feature(featureInfo, *this), [](Feature* f) {delete f; });
            // Only add visible features to the feature list
            if ( VmbFeatureVisibilityInvisible != featureInfo.visibility )
            {
                m_pImpl->m_features[name] = tmpFeature;
            }
            rFeature = std::move(tmpFeature);
        }
        catch (std::bad_alloc const&)
        {
            res = VmbErrorResources;
        }
    }

    return static_cast<VmbErrorType>(res);
}

VmbErrorType FeatureContainer::GetFeatures( FeaturePtr *pFeatures, VmbUint32_t &rnSize )
{
    VmbError_t res;

    if ( nullptr == m_pImpl->m_handle )
    {
        return VmbErrorDeviceNotOpen;
    }

    // Feature list is static and therefore needs to be fetched only once per lifetime
    if ( false == m_pImpl->m_bAllFeaturesFetched )
    {
        std::vector<VmbFeatureInfo_t> featureInfoList;

        res = VmbFeaturesList( m_pImpl->m_handle, nullptr, 0, &rnSize, sizeof(VmbFeatureInfo_t) );
        if ( 0 == rnSize || VmbErrorSuccess != res )
        {
            return (VmbErrorType)res;
        }

        featureInfoList.resize( rnSize );
        res = VmbFeaturesList( m_pImpl->m_handle, &featureInfoList[0], rnSize, &rnSize, sizeof(VmbFeatureInfo_t) );
        if ( VmbErrorSuccess != res )
        {
            return (VmbErrorType)res;
        }

        for (   std::vector<VmbFeatureInfo_t>::iterator iter = featureInfoList.begin();
                featureInfoList.end() != iter;
                ++iter )
        {
            std::string strName = iter->name;
            if ( 0 != strName.length() )
            {
                if ( SP_ISNULL( m_pImpl->m_features[strName] ))
                {
                    m_pImpl->m_features[strName] = FeaturePtr( new Feature(*iter, *this), [](Feature* f) {delete f; });
                }
            }
        }

        m_pImpl->m_bAllFeaturesFetched = true;
    }
    else // Features have been fetched before
    {
        res = VmbErrorSuccess;
    }

    if ( VmbErrorSuccess == res )
    {
        if ( nullptr == pFeatures )
        {
            rnSize = (VmbUint32_t)m_pImpl->m_features.size();
            return VmbErrorSuccess;
        }
        else if ( m_pImpl->m_features.size() <= rnSize )
        {
            VmbUint32_t i = 0;
            for (   FeaturePtrMap::iterator iter = m_pImpl->m_features.begin();
                    m_pImpl->m_features.end() != iter;
                    ++iter, ++i )
            {
                pFeatures[i] = iter->second;
            }
            rnSize = (VmbUint32_t)m_pImpl->m_features.size();
            return VmbErrorSuccess;
        }
        else
        {
            return VmbErrorMoreData;
        }
    }
    else
    {
        return (VmbErrorType)res;
    }
}

VmbHandle_t FeatureContainer::GetHandle() const noexcept
{
    return m_pImpl->m_handle;
}

void FeatureContainer::SetHandle( const VmbHandle_t handle )
{
    if ( nullptr == handle )
    {
        Reset();
        RevokeHandle();
    }
    else
    {
        m_pImpl->m_handle = handle;
    }
}

// Sets the C handle to null
void FeatureContainer::RevokeHandle() noexcept
{
    m_pImpl->m_handle = nullptr;
}

// Sets the back reference to feature container that each feature holds to null
// and resets all known features
void FeatureContainer::Reset()
{
    for (   FeaturePtrMap::iterator iter = m_pImpl->m_features.begin();
            m_pImpl->m_features.end() != iter;
            ++iter)
    {
        SP_ACCESS( iter->second )->ResetFeatureContainer();
    }

    m_pImpl->m_features.clear();
    m_pImpl->m_bAllFeaturesFetched = false;
}

}  // namespace VmbCPP
