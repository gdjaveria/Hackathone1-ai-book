import React, { useState } from 'react';
import Chatbot from '@site/src/components/Chatbot';
import './styles.css';

const FloatingChatWidget = () => {
  const [isOpen, setIsOpen] = useState(false);

  return (
    <>
      {!isOpen && (
        <button
          className="floating-chat-button"
          onClick={() => setIsOpen(true)}
          title="Open AI Assistant"
        >
          💬
        </button>
      )}

      {isOpen && (
        <div className="floating-chat-container">
          <div className="floating-chat-header">
            <h4>AI Assistant</h4>
            <button
              className="close-chat-button"
              onClick={() => setIsOpen(false)}
            >
              ×
            </button>
          </div>
          <div className="chat-content">
            <Chatbot />
          </div>
        </div>
      )}
    </>
  );
};

export default FloatingChatWidget;