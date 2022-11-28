const input = document.querySelector('input');
  const preview = document.querySelector('.preview');

  input.addEventListener('change', showTextFile);

  function showTextFile() {
    const selectedFiles = input.files;
    const list = document.createElement('ul');
    preview.appendChild(list);
    
    for(const file of selectedFiles) {
      const listItem = document.createElement('li');

      if(validFileType(file)) {
        const summary = document.createElement('div');
        summary.textContent = `파일명 : ${file.name}, 파일 크기 : ${returnFileSize(file.size)}.`;
        const textContents = document.createElement('div');
        
        let reader = new FileReader();
        reader.onload = function () {
          textContents.innerText = reader.result;
        };
        reader.readAsText(file, "UTF-8");
        
        listItem.appendChild(summary);
        listItem.appendChild(textContents);
      } else {
        const message = document.createElement('div');
        message.textContent = `파일명 ${file.name}: .txt 파일을 선택하세요`;
        listItem.appendChild(message);
      }

      list.appendChild(listItem);
    }
  }

  const fileTypes = [
    'text/plain',
  ];

  function validFileType(file) {
    return fileTypes.includes(file.type);
  }

  function returnFileSize(number) {
    if(number < 1024) {
      return number + 'bytes';
    } else if(number > 1024 && number < 1048576) {
      return (number/1024).toFixed(1) + 'KB';
    } else if(number > 1048576) {
      return (number/1048576).toFixed(1) + 'MB';
    }
  }