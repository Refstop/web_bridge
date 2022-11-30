// Copyright (c) 2017 Intel Corporation. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

'use strict';

// express server 생성, html은 url에 적혀있다
const express = require('express');
const app = express();

app.use(express.static('.'));

app.listen(3000);
console.log('The web server started on http://localhost:3000');

// app.get('/', function(req, res){
//     res.send('Hello World');
// }); // 임시 라우터 작성
// 브라우저로부터 request가 왔을때(접속시를 말하는 듯) 본 서버가 취해야 할 행동(여기에선 Hello world 출력)