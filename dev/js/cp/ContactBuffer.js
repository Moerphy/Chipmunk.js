/*


struct cpContactBufferHeader {
	cpTimestamp stamp;
	cpContactBufferHeader *next;
	unsigned int numContacts;
};

#define CP_CONTACTS_BUFFER_SIZE ((CP_BUFFER_BYTES - sizeof(cpContactBufferHeader))/sizeof(cpContact))
typedef struct cpContactBuffer {
	cpContactBufferHeader header;
	cpContact contacts[CP_CONTACTS_BUFFER_SIZE];
} cpContactBuffer;


 */

define([], function(){
  "use strict";
  var ContactBuffer = function(stamp, headerSplice){
    this.stamp = stamp;
    this.next = headerSplice?headerSplice.next:headerSplice;
    this.numContacts = 0;
    this.contacts = [];
    return this;
  };

  return ContactBuffer;
});


