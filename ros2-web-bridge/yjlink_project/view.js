var ROS2D = ROS2D || {
    REVISION: "0.9.0"
};
createjs
    .Stage
    .prototype
    .globalToRos = function (a, b) {
    var c = (a - this.x) / this.scaleX,
        d = (this.y - b) / this.scaleY;
    return new ROSLIB.Vector3({x: c, y: d})
},
createjs
    .Stage
    .prototype
    .rosToGlobal = function (a) {
    var b = a.x * this.scaleX + this.x,
        c = a.y * this.scaleY + this.y;
    return {x: b, y: c}
},
createjs
    .Stage
    .prototype
    .rosQuaternionToGlobalTheta = function (a) {
    var b = a.w,
        c = a.x,
        d = a.y,
        e = a.z;
    return 180 * -Math.atan2(2 * (b * e + c * d), 1 - 2 * (d * d + e * e)) / Math.PI
},
ROS2D.ImageMap = function (a) {
    a = a || {};
    var b = a.message,
        c = a.image;
    this.pose = new ROSLIB.Pose({position: b.origin.position, orientation: b.origin.orientation}),
    this.width = b.width,
    this.height = b.height,
    createjs.Bitmap.call(this, c),
    this.y = -this.height * b.resolution,
    this.scaleX = b.resolution,
    this.scaleY = b.resolution,
    this.width *= this.scaleX,
    this.height *= this.scaleY,
    this.x += this
        .pose
        .position
        .x,
    this.y -= this
        .pose
        .position
        .y
},
ROS2D
    .ImageMap
    .prototype
    .__proto__ = createjs.Bitmap.prototype,
ROS2D.ImageMapClient = function (a) {
    var b = this;
    a = a || {};
    var c = a.ros,
        d = a.topic || "/map_metadata";
    this.image = a.image,
    this.rootObject = a.rootObject || new createjs.Container,
    this.currentImage = new createjs.Shape;
    var e = new ROSLIB.Topic({ros: c, name: d, messageType: "nav_msgs/MapMetaData"});
    e.subscribe(function (a) {
        e.unsubscribe(),
        b.currentImage = new ROS2D.ImageMap({message: a, image: b.image}),
        b.rootObject.addChild(b.currentImage),
        b.rootObject.addChild(new ROS2D.Grid({size: 1})),
        b.emit("change")
    })
},
ROS2D
    .ImageMapClient
    .prototype
    .__proto__ = EventEmitter2.prototype,
ROS2D.OccupancyGrid = function (a) {
    a = a || {};
    var b = a.message,
        c = document.createElement("canvas"),
        d = c.getContext("2d");
    this.pose = new ROSLIB.Pose({
        position: b
            .info
            .origin
            .position, orientation: b
            .info
            .origin
            .orientation
    }),
    this.width = b.info.width,
    this.height = b.info.height,
    c.width = this.width,
    c.height = this.height;
    for (var e = d.createImageData(this.width, this.height), f = 0; f < this.height; f++) 
        for (var g = 0; g < this.width; g++) {
            var h,
                i = g + (this.height - f - 1) * this.width,
                j = b.data[i];
            h = 100 === j
                ? 0
                : 0 === j
                    ? 255
                    : 127;
            var k = 4 * (g + f * this.width);
            e.data[k] = h,
            e.data[++ k] = h,
            e.data[++ k] = h,
            e.data[++ k] = 255
        }
    
    d.putImageData(e, 0, 0),
    createjs.Bitmap.call(this, c),
    this.y = -this.height * b.info.resolution,
    this.scaleX = b.info.resolution,
    this.scaleY = b.info.resolution,
    this.width *= this.scaleX,
    this.height *= this.scaleY,
    this.x += this
        .pose
        .position
        .x,
    this.y -= this
        .pose
        .position
        .y
},
ROS2D
    .OccupancyGrid
    .prototype
    .__proto__ = createjs.Bitmap.prototype,
ROS2D.OccupancyGridClient = function (a) {
    var b = this;
    a = a || {};
    var c = a.ros,
        d = a.topic || "/map";
    this.continuous = a.continuous,
    this.rootObject = a.rootObject || new createjs.Container,
    this.currentGrid = new createjs.Shape,
    this.rootObject.addChild(this.currentGrid),
    this.rootObject.addChild(new ROS2D.Grid({size: 1}));
    var e = new ROSLIB.Topic({ros: c, name: d, messageType: "nav_msgs/OccupancyGrid", compression: "png"});
    e.subscribe(function (a) {
        var c = null;
        b.currentGrid && (c = b.rootObject.getChildIndex(b.currentGrid), b.rootObject.removeChild(b.currentGrid)),
        b.currentGrid = new ROS2D.OccupancyGrid({message: a}),
        null !== c
            ? b.rootObject.addChildAt(b.currentGrid, c)
            : b.rootObject.addChild(b.currentGrid),
        b.emit("change"),
        b.continuous || e.unsubscribe()
    })
},
ROS2D
    .OccupancyGridClient
    .prototype
    .__proto__ = EventEmitter2.prototype,
ROS2D.OccupancyGridSrvClient = function (a) {
    var b = this;
    a = a || {};
    var c = a.ros,
        d = a.service || "/static_map";
    this.rootObject = a.rootObject || new createjs.Container,
    this.currentGrid = null;
    var e = new ROSLIB.Service({ros: c, name: d, serviceType: "nav_msgs/GetMap", compression: "png"});
    e.callService(new ROSLIB.ServiceRequest, function (a) {
        b.currentGrid && b.rootObject.removeChild(b.currentGrid),
        b.currentGrid = new ROS2D.OccupancyGrid({message: a.map}),
        b.rootObject.addChild(b.currentGrid),
        b.emit("change", b.currentGrid)
    })
},
ROS2D
    .OccupancyGridSrvClient
    .prototype
    .__proto__ = EventEmitter2.prototype,
ROS2D.ArrowShape = function (a) {
    var b = this;
    a = a || {};
    var c = a.size || 10,
        d = a.strokeSize || 3,
        e = a.strokeColor || createjs.Graphics.getRGB(0, 0, 0),
        f = a.fillColor || createjs.Graphics.getRGB(255, 0, 0),
        g = a.pulse,
        h = new createjs.Graphics,
        i = c / 3,
        j = 2 * i / 3;
    if (h.setStrokeStyle(d), h.beginStroke(e), h.moveTo(0, 0), h.lineTo(c - i, 0), h.beginFill(f), h.moveTo(c, 0), h.lineTo(c - i, j / 2), h.lineTo(c - i, - j / 2), h.closePath(), h.endFill(), h.endStroke(), createjs.Shape.call(this, h), g) {
        var k = 0,
            l = !0;
        createjs.Ticker.addEventListener("tick", function () {
            l
                ? (b.scaleX *= 1.035, b.scaleY *= 1.035, l =++ k < 10)
                : (b.scaleX /= 1.035, b.scaleY /= 1.035, l =-- k < 0)
        })
    }
},
ROS2D
    .ArrowShape
    .prototype
    .__proto__ = createjs.Shape.prototype,
ROS2D.Grid = function (a) {
    a = a || {};
    var b = a.size || 10,
        c = a.cellSize || .1,
        d = a.lineWidth || .001,
        e = new createjs.Graphics;
    e.setStrokeStyle(5 * d),
    e.beginStroke(createjs.Graphics.getRGB(0, 0, 0)),
    e.beginFill(createjs.Graphics.getRGB(255, 0, 0)),
    e.moveTo(- b * c, 0),
    e.lineTo(b * c, 0),
    e.moveTo(0, - b * c),
    e.lineTo(0, b * c),
    e.endFill(),
    e.endStroke(),
    e.setStrokeStyle(d),
    e.beginStroke(createjs.Graphics.getRGB(0, 0, 0)),
    e.beginFill(createjs.Graphics.getRGB(255, 0, 0));
    for (var f = - b; b >= f; f++) 
        e.moveTo(- b * c, f * c),
        e.lineTo(b * c, f * c),
        e.moveTo(f * c, - b * c),
        e.lineTo(f * c, b * c);
    
    e.endFill(),
    e.endStroke(),
    createjs.Shape.call(this, e)
},
ROS2D
    .Grid
    .prototype
    .__proto__ = createjs.Shape.prototype,
ROS2D.NavigationArrow = function (a) {
    var b = this;
    a = a || {};
    var c = a.size || 10,
        d = a.strokeSize || 3,
        e = a.strokeColor || createjs.Graphics.getRGB(0, 0, 0),
        f = a.fillColor || createjs.Graphics.getRGB(255, 0, 0),
        g = a.pulse,
        h = new createjs.Graphics;
    if (h.setStrokeStyle(d), h.moveTo(- c / 2, - c / 2), h.beginStroke(e), h.beginFill(f), h.lineTo(c, 0), h.lineTo(- c / 2, c / 2), h.closePath(), h.endFill(), h.endStroke(), createjs.Shape.call(this, h), g) {
        var i = 0,
            j = !0;
        createjs.Ticker.addEventListener("tick", function () {
            j
                ? (b.scaleX *= 1.035, b.scaleY *= 1.035, j =++ i < 10)
                : (b.scaleX /= 1.035, b.scaleY /= 1.035, j =-- i < 0)
        })
    }
},
ROS2D
    .NavigationArrow
    .prototype
    .__proto__ = createjs.Shape.prototype,
ROS2D.NavigationImage = function (a) {
    var b = this;
    a = a || {};
    var c = a.size || 10,
        d = a.image,
        e = a.pulse,
        f = a.alpha || 1,
        g = {},
        h = function () {
            createjs.Bitmap.call(b, j);
            var a = i(c);
            if (b.alpha = f, b.scaleX = a, b.scaleY = a, b.regY = b.image.height / 2, b.regX = b.image.width / 2, g.rotation = b.rotation, Object.defineProperty(b, "rotation", {
                get: function () {
                    return g.rotation + 90
                },
                set: function (a) {
                    g.rotation = a
                }
            }), e) {
                var d = 0,
                    h = !0,
                    k = 1.02;
                createjs.Ticker.addEventListener("tick", function () {
                    h
                        ? (b.scaleX *= k, b.scaleY *= k, h =++ d < 10)
                        : (b.scaleX /= k, b.scaleY /= k, h =-- d < 0)
                })
            }
        },
        i = function (a) {
            return a / j.width
        },
        j = new Image;
    j.onload = h,
    j.src = d
},
ROS2D
    .NavigationImage
    .prototype
    .__proto__ = createjs.Bitmap.prototype,
ROS2D.PathShape = function (a) {
    a = a || {};
    var b = a.path;
    if (this.strokeSize = a.strokeSize || 3, this.strokeColor = a.strokeColor || createjs.Graphics.getRGB(0, 0, 0), this.graphics = new createjs.Graphics, null !== b && "undefined" != typeof b) {
        this.graphics.setStrokeStyle(this.strokeSize),
        this.graphics.beginStroke(this.strokeColor),
        this.graphics.moveTo(b
            .poses[0]
            .pose
            .position
            .x / this.scaleX, b
            .poses[0]
            .pose
            .position
            .y / -this.scaleY);
        for (var c = 1; c < b.poses.length; ++ c) 
            this.graphics.lineTo(b
                .poses[c]
                .pose
                .position
                .x / this.scaleX, b
                .poses[c]
                .pose
                .position
                .y / -this.scaleY);
        
        this.graphics.endStroke()
    }
    createjs.Shape.call(this, this.graphics)
},
ROS2D
    .PathShape
    .prototype
    .setPath = function (a) {
    if (this.graphics.clear(), null !== a && "undefined" != typeof a) {
        this.graphics.setStrokeStyle(this.strokeSize),
        this.graphics.beginStroke(this.strokeColor),
        this.graphics.moveTo(a
            .poses[0]
            .pose
            .position
            .x / this.scaleX, a
            .poses[0]
            .pose
            .position
            .y / -this.scaleY);
        for (var b = 1; b < a.poses.length; ++ b) 
            this.graphics.lineTo(a
                .poses[b]
                .pose
                .position
                .x / this.scaleX, a
                .poses[b]
                .pose
                .position
                .y / -this.scaleY);
        
        this.graphics.endStroke()
    }
},
ROS2D
    .PathShape
    .prototype
    .__proto__ = createjs.Shape.prototype,
ROS2D.PolygonMarker = function (a) {
    a = a || {},
    this.lineSize = a.lineSize || 3,
    this.lineColor = a.lineColor || createjs.Graphics.getRGB(0, 0, 255, .66),
    this.pointSize = a.pointSize || 10,
    this.pointColor = a.pointColor || createjs.Graphics.getRGB(255, 0, 0, .66),
    this.fillColor = a.pointColor || createjs.Graphics.getRGB(0, 255, 0, .33),
    this.lineCallBack = a.lineCallBack,
    this.pointCallBack = a.pointCallBack,
    this.pointContainer = new createjs.Container,
    this.lineContainer = new createjs.Container,
    this.fillShape = new createjs.Shape,
    createjs.Container.call(this),
    this.addChild(this.fillShape),
    this.addChild(this.lineContainer),
    this.addChild(this.pointContainer)
},
ROS2D
    .PolygonMarker
    .prototype
    .createLineShape = function (a, b) {
    var c = new createjs.Shape;
    this.editLineShape(c, a, b);
    var d = this;
    return c.addEventListener("mousedown", function (a) {
        null !== d.lineCallBack && "undefined" != typeof d.lineCallBack && d.lineCallBack("mousedown", a, d.lineContainer.getChildIndex(a.target))
    }),
    c
},
ROS2D
    .PolygonMarker
    .prototype
    .editLineShape = function (a, b, c) {
    a.graphics.clear(),
    a.graphics.setStrokeStyle(this.lineSize),
    a.graphics.beginStroke(this.lineColor),
    a.graphics.moveTo(b.x, b.y),
    a.graphics.lineTo(c.x, c.y)
},
ROS2D
    .PolygonMarker
    .prototype
    .createPointShape = function (a) {
    var b = new createjs.Shape;
    b.graphics.beginFill(this.pointColor),
    b.graphics.drawCircle(0, 0, this.pointSize),
    b.x = a.x,
    b.y = - a.y;
    var c = this;
    return b.addEventListener("mousedown", function (a) {
        null !== c.pointCallBack && "undefined" != typeof c.pointCallBack && c.pointCallBack("mousedown", a, c.pointContainer.getChildIndex(a.target))
    }),
    b
},
ROS2D
    .PolygonMarker
    .prototype
    .addPoint = function (a) {
    var b = this.createPointShape(a);
    this.pointContainer.addChild(b);
    var c = this.pointContainer.getNumChildren();
    if (2 > c) 
     else if (3 > c) {
        var d = this.createLineShape(this.pointContainer.getChildAt(c - 2), b);
        this.lineContainer.addChild(d)
    }
    if (c > 2 && this.editLineShape(this.lineContainer.getChildAt(c - 2), this.pointContainer.getChildAt(c - 2), b), c > 1) {
        var e = this.createLineShape(b, this.pointContainer.getChildAt(0));
        this.lineContainer.addChild(e)
    }
    this.drawFill()
},
ROS2D
    .PolygonMarker
    .prototype
    .remPoint = function (a) {
    var b;
    b = a instanceof createjs.Shape
        ? this.pointContainer.getChildIndex(a)
        : a;
    var c = this.pointContainer.getNumChildren();
    2 > c || (
        3 > c
            ? this.lineContainer.removeAllChildren()
            : (this.editLineShape(this.lineContainer.getChildAt((b - 1 + c) % c), this.pointContainer.getChildAt((b - 1 + c) % c), this.pointContainer.getChildAt((b + 1) % c)), this.lineContainer.removeChildAt(b))
    ),
    this.pointContainer.removeChildAt(b),
    this.drawFill()
},
ROS2D
    .PolygonMarker
    .prototype
    .movePoint = function (a, b) {
    var c,
        d;
    a instanceof createjs.Shape
        ? (c = this.pointContainer.getChildIndex(a), d = a)
        : (c = a, d = this.pointContainer.getChildAt(c)),
    d.x = b.x,
    d.y = - b.y;
    var e = this.pointContainer.getNumChildren();
    if (e > 1) {
        var f = this.lineContainer.getChildAt((c - 1 + e) % e);
        this.editLineShape(f, this.pointContainer.getChildAt((c - 1 + e) % e), d);
        var g = this.lineContainer.getChildAt(c);
        this.editLineShape(g, d, this.pointContainer.getChildAt((c + 1) % e))
    }
    this.drawFill()
},
ROS2D
    .PolygonMarker
    .prototype
    .splitLine = function (a) {
    var b,
        c;
    a instanceof createjs.Shape
        ? (b = this.lineContainer.getChildIndex(a), c = a)
        : (b = a, c = this.lineContainer.getChildAt(b));
    var d = this.pointContainer.getNumChildren(),
        e = this
            .pointContainer
            .getChildAt(b)
            .x,
        f = this
            .pointContainer
            .getChildAt(b)
            .y,
        g = this
            .pointContainer
            .getChildAt((b + 1) % d)
            .x,
        h = this
            .pointContainer
            .getChildAt((b + 1) % d)
            .y,
        i = (e + g) / 2,
        j = (f + h) / 2,
        k = new ROSLIB.Vector3({
            x: i,
            y: - j
        }),
        l = this.createPointShape(k);
    this.pointContainer.addChildAt(l, b + 1),
    ++ d;
    var m = this.createLineShape(l, this.pointContainer.getChildAt((b + 2) % d));
    this.lineContainer.addChildAt(m, b + 1),
    this.editLineShape(c, this.pointContainer.getChildAt(b), l),
    this.drawFill()
},
ROS2D
    .PolygonMarker
    .prototype
    .drawFill = function () {
    var a = this.pointContainer.getNumChildren();
    if (a > 2) {
        var b = this.fillShape.graphics;
        b.clear(),
        b.setStrokeStyle(0),
        b.moveTo(this
            .pointContainer
            .getChildAt(0)
            .x, this
            .pointContainer
            .getChildAt(0)
            .y),
        b.beginStroke(),
        b.beginFill(this.fillColor);
        for (var c = 1; a > c; ++ c) 
            b.lineTo(this
                .pointContainer
                .getChildAt(c)
                .x, this
                .pointContainer
                .getChildAt(c)
                .y);
        
        b.closePath(),
        b.endFill(),
        b.endStroke()
    } else 
        this
            .fillShape
            .graphics
            .clear()
    
},
ROS2D
    .PolygonMarker
    .prototype
    .__proto__ = createjs.Container.prototype,
ROS2D.TraceShape = function (a) {
    a = a || {};
    var b = a.pose;
    this.strokeSize = a.strokeSize || 3,
    this.strokeColor = a.strokeColor || createjs.Graphics.getRGB(0, 0, 0),
    this.maxPoses = a.maxPoses || 100,
    this.minDist = a.minDist || .05,
    this.minDist = this.minDist * this.minDist,
    this.poses = [],
    this.graphics = new createjs.Graphics,
    this.graphics.setStrokeStyle(this.strokeSize),
    this.graphics.beginStroke(this.strokeColor),
    null !== b && "undefined" != typeof b && this.poses.push(b),
    createjs.Shape.call(this, this.graphics)
},
ROS2D
    .TraceShape
    .prototype
    .addPose = function (a) {
        var b = this.poses.length - 1;
        if (0 > b) 
            this.poses.push(a),
            this.graphics.moveTo(a.position.x / this.scaleX, a.position.y / -this.scaleY);
         else {
            var c = this
                    .poses[b]
                    .position
                    .x,
                d = this
                    .poses[b]
                    .position
                    .y,
                e = a.position.x - c,
                f = a.position.y - d;
            e * e + f * f > this.minDist && (this.graphics.lineTo(a.position.x / this.scaleX, a.position.y / -this.scaleY), this.poses.push(a))
        }
        this.maxPoses > 0 && this.maxPoses<this.poses.length && this.popFront()
}, ROS2D.TraceShape.prototype.popFront = function() {
    if (this.poses.length> 0
    ) {
        this.poses.shift(),
        this.graphics.clear(),
        this.graphics.setStrokeStyle(this.strokeSize),
        this.graphics.beginStroke(this.strokeColor),
        this.graphics.lineTo(this
            .poses[0]
            .position
            .x / this.scaleX, this
            .poses[0]
            .position
            .y / -this.scaleY);
        for (var a = 1; a < this.poses.length; ++ a) 
            this.graphics.lineTo(this
                .poses[a]
                .position
                .x / this.scaleX, this
                .poses[a]
                .position
                .y / -this.scaleY)
        
    }
},
ROS2D
    .TraceShape
    .prototype
    .__proto__ = createjs.Shape.prototype,
ROS2D.PanView = function (a) {
    a = a || {},
    this.rootObject = a.rootObject,
    this.rootObject instanceof createjs.Stage
        ? this.stage = this.rootObject
        : this.stage = this.rootObject.getStage(),
    this.startPos = new ROSLIB.Vector3
},
ROS2D
    .PanView
    .prototype
    .startPan = function (a, b) {
    this.startPos.x = a,
    this.startPos.y = b
},
ROS2D
    .PanView
    .prototype
    .pan = function (a, b) {
    this.stage.x += a - this.startPos.x,
    this.startPos.x = a,
    this.stage.y += b - this.startPos.y,
    this.startPos.y = b
},
ROS2D.Viewer = function (a) {
    a = a || {};
    var b = a.divID;
    this.width = a.width,
    this.height = a.height;
    var c = a.background || "#111111",
        d = document.createElement("canvas");
    d.width = this.width,
    d.height = this.height,
    d.style.background = c,
    document.getElementById(b).appendChild(d),
    this.scene = new createjs.Stage(d),
    this.scene.y = this.height,
    document.getElementById(b).appendChild(d),
    createjs.Ticker.setFPS(30),
    createjs.Ticker.addEventListener("tick", this.scene)
},
ROS2D
    .Viewer
    .prototype
    .addObject = function (a) {
    this.scene.addChild(a)
},
ROS2D
    .Viewer
    .prototype
    .scaleToDimensions = function (a, b) {
    this.scene.x = "undefined" != typeof this.scene.x_prev_shift
        ? this.scene.x_prev_shift
        : this.scene.x,
    this.scene.y = "undefined" != typeof this.scene.y_prev_shift
        ? this.scene.y_prev_shift
        : this.scene.y,
    this.scene.scaleX = this.width / a,
    this.scene.scaleY = this.height / b
},
ROS2D
    .Viewer
    .prototype
    .shift = function (a, b) {
    this.scene.x_prev_shift = this.scene.x,
    this.scene.y_prev_shift = this.scene.y,
    this.scene.x -= a * this.scene.scaleX,
    this.scene.y += b * this.scene.scaleY
},
ROS2D.ZoomView = function (a) {
    a = a || {},
    this.rootObject = a.rootObject,
    this.minScale = a.minScale || .001,
    this.rootObject instanceof createjs.Stage
        ? this.stage = this.rootObject
        : this.stage = this.rootObject.getStage(),
    this.center = new ROSLIB.Vector3,
    this.startShift = new ROSLIB.Vector3,
    this.startScale = new ROSLIB.Vector3
},
ROS2D
    .ZoomView
    .prototype
    .startZoom = function (a, b) {
    this.center.x = a,
    this.center.y = b,
    this.startShift.x = this.stage.x,
    this.startShift.y = this.stage.y,
    this.startScale.x = this.stage.scaleX,
    this.startScale.y = this.stage.scaleY
},
ROS2D
    .ZoomView
    .prototype
    .zoom = function (a) {
    this.startScale.x * a < this.minScale && (a = this.minScale / this.startScale.x),
    this.startScale.y * a < this.minScale && (a = this.minScale / this.startScale.y),
    this.stage.scaleX = this.startScale.x * a,
    this.stage.scaleY = this.startScale.y * a,
    this.stage.x = this.startShift.x -(this.center.x - this.startShift.x) * (this.stage.scaleX / this.startScale.x - 1),
    this.stage.y = this.startShift.y -(this.center.y - this.startShift.y) * (this.stage.scaleY / this.startScale.y - 1)
};