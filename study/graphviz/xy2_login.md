##������Ϸ����

```graphviz
digraph startgame {
    label="��Ϸ��Դ��������"
    rankdir="TB"
    start[label="������Ϸ" shape=circle style=filled]
    ifwifi[label="���绷���ж��Ƿ� WIFI" shape=diamond]
    needupdate[label="�Ƿ�����Դ��Ҫ����" shape=diamond]
    startslientdl[label="��Ĭ����" shape=box]
    enterhall[label="������Ϸ����" shape=box]
    enterroom[label="���뷿��" shape=box]
    resourceuptodate[label="��Դ������" shape=diamond]
    startplay[label="������Ϸ" shape=circle fillcolor=blue]
    warning[label="��������Ƿ����" shape=diamond]
    startdl[label="�������ؽ���" shape=box]
    //{rank=same; needupdate, enterhall}
    {shape=diamond; ifwifi, needupdate}
    start -> ifwifi
    ifwifi->needupdate[label="��"]
    ifwifi->enterhall[label="��"]
    needupdate->startslientdl[label="��"]
    startslientdl->enterhall
    needupdate->enterhall[label="��"]
    enterhall -> enterroom
    enterroom -> resourceuptodate
    resourceuptodate -> warning[label="��"]
    resourceuptodate -> startplay[label="��"]
    warning -> startdl[label="ȷ������"]
    warning -> enterhall[label="ȡ������"]
    startdl -> enterhall[label="ȡ������"]
    startdl -> startplay[label="�������"]
}
}
```