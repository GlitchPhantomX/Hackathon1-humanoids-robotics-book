import React from 'react';
import OriginalNavbar from '@theme-original/Navbar';
import Banner from '@site/src/components/Banner';

export default function Navbar(props) {
  return (
    <>
      <Banner />
      <OriginalNavbar {...props} />
    </>
  );
}