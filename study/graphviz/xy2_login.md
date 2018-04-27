##启动游戏流程

```graphviz
digraph startgame {
    label="游戏资源更新流程"
    rankdir="TB"
    start[label="启动游戏" shape=circle style=filled]
    ifwifi[label="网络环境判断是否 WIFI" shape=diamond]
    needupdate[label="是否有资源需要更新" shape=diamond]
    startslientdl[label="静默下载" shape=box]
    enterhall[label="进入游戏大厅" shape=box]
    enterroom[label="进入房间" shape=box]
    resourceuptodate[label="资源不完整" shape=diamond]
    startplay[label="正常游戏" shape=circle fillcolor=blue]
    warning[label="提醒玩家是否更新" shape=diamond]
    startdl[label="进入下载界面" shape=box]
    //{rank=same; needupdate, enterhall}
    {shape=diamond; ifwifi, needupdate}
    start -> ifwifi
    ifwifi->needupdate[label="是"]
    ifwifi->enterhall[label="否"]
    needupdate->startslientdl[label="是"]
    startslientdl->enterhall
    needupdate->enterhall[label="否"]
    enterhall -> enterroom
    enterroom -> resourceuptodate
    resourceuptodate -> warning[label="是"]
    resourceuptodate -> startplay[label="否"]
    warning -> startdl[label="确认下载"]
    warning -> enterhall[label="取消下载"]
    startdl -> enterhall[label="取消下载"]
    startdl -> startplay[label="下载完成"]
}
}
```