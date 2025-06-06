#
#  Metwork Project  (sip3-metawork)
# 

# Aglie-X Piper IK

Aglie-X Pipe を IK で MQTT 経由で Joint 情報を送るコード

pnpm install 
pnpm dev-https

2025/4/5 に next.js 15.2.4 に変更
MetaworkMQTT プロトコルに対応中。なんとか動きそう。

ただし、実ロボットに適用するには、まだ危険。



Chrome ブラウザで動かすには、WebXR のプラグイン（Immersive Web Emulator)が必要です。
https://chromewebstore.google.com/detail/immersive-web-emulator/cgffilbpcibhmcfbgggfhfolhkfbhmik?hl=ja

また、MQTTの設定などが static に書いてあるので修正が必要です。

MQTT 側で動作するマネージャもプロトタイプを作りました

https://github.com/nkawa/MetaworkMQTT

（なお sora2.uclab.jp であれば、すでに MetaworkMQTT が入れてあるので、入れなくても動作するはず）

A-Frame 1.7.1 にあげようとしたら、 Immersive Web Emulator がうまく動かないので、戻しました。
