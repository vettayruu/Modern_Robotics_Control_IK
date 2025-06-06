

function getOrSetUUID() {
    const cookieName = "sip3m_uuid";
    
    // Cookieを取得する関数
    function getCookie(name) {
        const cookies = document.cookie.split("; ");
        for (let cookie of cookies) {
            let [key, value] = cookie.split("=");
            if (key === name) {
                return value;
            }
        }
        return null;
    }

    // Cookieに保存する関数
    function setCookie(name, value, days) {
        let expires = "";
        if (days) {
            const date = new Date();
            date.setTime(date.getTime() + days * 24 * 60 * 60 * 1000);
            expires = "; expires=" + date.toUTCString();
        }
        document.cookie = name + "=" + value + "; path=/" + expires;
    }

    // UUIDを取得または新規生成
    let uuid = getCookie(cookieName);
    if (!uuid) {
        uuid = crypto.randomUUID();
        setCookie(cookieName, uuid, 365); // 1年間保存
    }

    return uuid;
}

// UUIDを取得または設定
export const userUUID = getOrSetUUID();
console.log("User UUID:", userUUID);