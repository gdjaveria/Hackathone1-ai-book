import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import FloatingChatWidget from '@site/src/components/FloatingChatWidget';

export default function Layout(props) {
  return (
    <>
      <OriginalLayout {...props} />
      <FloatingChatWidget />
    </>
  );
}