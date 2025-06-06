import React from 'react';

const Assets = (props)=>{
  const path = (props.viewer || props.monitor) ? "../" : "";
  return (
    <a-assets>
      {/*Model*/}
      <a-asset-items id="base" src={`${path}base_link.gltf`} ></a-asset-items>
      <a-asset-items id="j1" src={`${path}link1.gltf`} ></a-asset-items>
      <a-asset-items id="j2" src={`${path}link2.gltf`} ></a-asset-items>
      <a-asset-items id="j3" src={`${path}link3.gltf`} ></a-asset-items>
      <a-asset-items id="j4" src={`${path}link4.gltf`} ></a-asset-items>
      <a-asset-items id="j5" src={`${path}link5.gltf`} ></a-asset-items>
      <a-asset-items id="j6" src={`${path}link6.gltf`} ></a-asset-items>
      <a-asset-items id="j6_1" src={`${path}link7.gltf`} ></a-asset-items>
      <a-asset-items id="j6_2" src={`${path}link8.gltf`} ></a-asset-items>
    </a-assets>
  )
}

export default Assets;